#include <Servo.h>
#include "MPU9250.h"
#include <Wire.h>
#include <VL53L0X.h>
#include <LiquidCrystal.h>

#define steerServoPin 13
#define g9ServoPin 11
#define sonicTrig 7
#define sonicEcho 8
#define ESCPin 9
#define redLEDPin 52
#define blueLEDPin 53

MPU9250 IMU(Wire, 0x68);
int status;

Servo steer_servo;
int steer_servo_pos;
double steer_value;
const int STEER_SERVO_MAX = 190;
const int STEER_SERVO_MIN = 80;
const int STEER_SERVO_MID = 135;

Servo g9_servo;
int g9_servo_pos;
int g9_high_pos = 171;
int g9_low_pos = 14;
bool g9_sweep_up = true;

Servo ESC;
int motor_throttle = 1500;  // Speed = 0
int desired_throttle = 1500;
const int ESC_HIGH = 2000;
const int ESC_LOW = 1500;

VL53L0X toF;

// (RS, E, D4, D5, D6, D7)
LiquidCrystal lcd = LiquidCrystal(2, 3, 4, 5, 6, 7);

// ***Vehicle Odometry***
// Position
double car_x_pos = 0.0;
double car_y_pos = 0.0;

// Linear Velocity
double car_x_vel = 0.0;
double car_y_vel = 0.0;

// Linear Acceleration
double car_x_accel = 0.0;
double car_y_accel = 0.0;
double car_z_accel = 0.0;

double x_callib = 0.0;
double y_callib = 0.0;

// Heading (Z-angle degrees)
double car_heading = 0.0;

// Z Angular Velocity (rad/s)
double car_z_ang_vel = 0.0;
double last_vel = 0.0;

// ***End Vehicle Odometry***

void updateOdometry(int time_ms){
  double time_s = time_ms/1000.0;

  // Update position
  car_x_pos = 0.5*car_x_accel*pow(time_s, 2) + car_x_vel*time_s + car_x_pos;
  car_y_pos = 0.5*car_y_accel*pow(time_s, 2) + car_y_vel*time_s + car_y_pos;

  // Update velocity
  car_x_vel = car_x_accel*time_s + car_x_vel;
  car_y_vel = car_y_accel*time_s + car_y_vel;
  
  // Velocity deadzones
  if (abs(car_x_vel) < 0.01){
    car_x_vel = 0;  
  }
  if (abs(car_y_vel) < 0.01){
    car_y_vel = 0;  
  }

  // Set velocities to 0 when motor is stopped
  if (abs(motor_throttle - 1500) < 50){
    car_x_vel = 0;
    car_y_vel = 0;  
  }

  IMU.readSensor();

  // Update acceleration
  car_x_accel = -IMU.getAccelX_mss() - x_callib;
  car_y_accel = -IMU.getAccelY_mss() - y_callib;
  car_z_accel = IMU.getAccelZ_mss() + 2.7;  // Account for z-axis error

  // Acceleration deadzones
  if (abs(car_x_accel) < 0.01){
    car_x_accel = 0;
  }
  if (abs(car_y_accel) < 0.01){
    car_y_accel = 0;
  }
  
  car_heading = fmod(car_z_ang_vel*180*time_s/PI + car_heading, 360);

  car_z_ang_vel = IMU.getGyroZ_rads();
  
  // Angular Velocity deadzone
  if (abs(car_z_ang_vel) < 0.01){
    car_z_ang_vel = 0.0;  
  }

  // Angular Velocity error correction
  car_z_ang_vel *= (90.0 / 85.0);
}

// ***Data & Data Structures***
// ToF Point Cloud
int x_coords[156];
int y_coords[156];
int coords_index = 0;

double destination_x = 2.0; // Meters
double destination_y = 0.0;
double destination_heading = 0.0; // Degrees

unsigned long time_ms;

enum carStates{
  FORWARD,
  OBSTACLE_STOP
};

int reverse_step_counter = 0; // Used as timing mechanism for reversing

enum carStates car_state = FORWARD;

// ***End Data***

// ***Setup Functions***
void setupIMU(){
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }
}

void setupESC(){
  // ESC Callibration
  ESC.writeMicroseconds(ESC_HIGH);
  ESC.writeMicroseconds(ESC_LOW);
}

void setupToF(){
  if (!toF.init()) {
    Serial.println(F("Failed to boot ToF Sensor"));
  }
  toF.init();
  toF.setTimeout(500);
  toF.startContinuous();
}

void callibrateAccelerometer(){
  /*
   * Callibrates X and Y accelerometer values to account for
   * any tilt in the IMU by taking averages when stationary. 
   * (Will cause some inaccuracy)
   */
  for (int i = 0; i < 20; ++i){
    IMU.readSensor();
    x_callib += -IMU.getAccelX_mss();
    y_callib += -IMU.getAccelY_mss();
    delay(20);
  }
  x_callib /= 20;
  y_callib /= 20;
}

// ***End Setup Functions***

void moveSteerServo(int new_pos){
  /*
   * Moves the steering servo to position 'new_pos', so long
   * as the provided position lies within mechanically
   * appropriate values.
   * @param new_pos: integer representing new servo angle
   * between 0 and 260 deg.
   */
  if (new_pos > STEER_SERVO_MAX){
    new_pos = STEER_SERVO_MAX;
  }
  else if (new_pos < STEER_SERVO_MIN){
    new_pos = STEER_SERVO_MIN;
  }
  
  if (new_pos != steer_servo_pos){
    steer_servo.write(map(new_pos, 0, 260, 0, 180));
    steer_servo_pos = new_pos;
  }
}

void steer(double amount){
  /*  
   *   Adjusts the steering servo angle to an offset 
   *   (45 * amount) from the middle.
   *   @param amount: floating-point value between -1 and 1.
   *   Values less than 0 steer left, values greater than 0 
   *   steer right.
   */
  if (amount < -1.0){
    amount = -1.0;  
  }
  else if (amount > 1.0){
    amount = 1.0;  
  }
  moveSteerServo(STEER_SERVO_MID + int(amount * 45));
}

long microsecondsToCm(long microseconds){
  /*
   * Converts time taken for an ultrasonic pulse in
   * microseconds to distance in cm.
   * @param microseconds: long representing pulse time.
   * @return: long representing cm distance.
   */
  return microseconds/29/2;  
}

long readSonicSensor(){
  /*  
   *   Returns distance read by Ultrasonic sensor in cm
   *   Based on: https://www.tutorialspoint.com/arduino/arduino_ultrasonic_sensor
   *   @return: long representing cm distance.
   */
  digitalWrite(sonicTrig, LOW);
  delayMicroseconds(2);
  digitalWrite(sonicTrig, HIGH);
  delayMicroseconds(5);
  digitalWrite(sonicTrig, LOW);

  long duration = pulseIn(sonicEcho, HIGH);

  return microsecondsToCm(duration);
}

int readToFSensor(){
  /*
   * Returns distance measured by time-of-flight sensor.
   * Added delay for out-of-range measurements, in order
   * to maintain time consistency.
   * @return: integer representing mm distance.
   */
  int range = toF.readRangeContinuousMillimeters();
  if (range > 1200 or range < 0){
    delay(12);  // Adjusted from 25ms
  }
  return range;
}

void adjustThrottle(int target_throttle, int step_size){
  /*
   * Adjusts the ESC throttle by provided increment to obtain
   * smooth acceleration and deceleration.
   * @param target_throttle: integer representing the throttle
   * the ESC needs to achieve.
   * @param step_size: integer representing the increment size
   * for the throttle.
   */
  
  // Adjusts step_size such that target_throttle is exactly attainable
  step_size = min(step_size, abs(motor_throttle - target_throttle));
  
  if (motor_throttle > target_throttle){
    motor_throttle -= step_size;
  }
  else if (motor_throttle < target_throttle){
    motor_throttle += step_size;  
  }
  ESC.writeMicroseconds(motor_throttle);
}

void updateForwardThrottle(){
  /*
   * Updates the car's 'desired_throttle' when it is in the
   * FORWARD state. Desired throttle is calculated as a 
   * function of distance to a stop point (the lesser of 
   * distance to the destination, and the nearest obstacle).
   */
  int target_distance = int(sqrt(pow(car_y_pos - destination_y, 2) + 
    pow(car_x_pos - destination_x, 2)) * 1000);
    
  int nearest_obstruction = nearestObstruction() - 100; // 100mm Offest
  int stop_point = min(target_distance, nearest_obstruction);

  if (stop_point >= 500){
    desired_throttle = -1800;  
  }
  else{
    if (stop_point < 0){
      stop_point = 0;  
    }
    desired_throttle = -(-6 * stop_point / 5000) * (stop_point - 1000) + 1500;
  }

  if (desired_throttle < 0){
    desired_throttle += 3000;
  }

  if (desired_throttle < 1250){
    desired_throttle = 1250;  // Limit throttle
  }

  if (desired_throttle > 1450){
    car_state = OBSTACLE_STOP;
  }
}

void updateReverseThrottle(){
  /*
   * Updates the car's 'desired_throttle' when it is in the
   * OBSTACLE_STOP state.
   */
  if (motor_throttle < 1450 and reverse_step_counter == 0){
    desired_throttle = 1750;
  }
  reverse_step_counter++;
  
  if (reverse_step_counter >= 125){
    desired_throttle = 1500;
    if (motor_throttle < 1550){
      car_state = FORWARD;
      car_heading = 90.0;
      reverse_step_counter = 0;
    }
  }
}

int nearestObstruction(){
  /*  
   *   Finds and returns the car's distance (in mm) to the nearest obstacle 
   *   which blocks its path, if any.
   *   @return: integer representing mm distance to obstacle.
   */
  int nearest = 5000; // Arbitrary

  for (int i = 0; i < 156; ++i){
    if (x_coords[i] > -210 and x_coords[i] < 210 and 
    y_coords[i] < nearest and y_coords[i] > 130){
      nearest = y_coords[i];
    }  
  }
  
  return nearest - 130; // Distance from front of car
}

void servoSweepStep(){
  /*
   * Moves the 9g servo to it's next position within the sweep;
   * incrementing servo angle when sweeping "up" and decrementing
   * when sweeping "down". The total oscillation angle depends on
   * the vehicle state.
   */
  if (car_state == OBSTACLE_STOP){
    g9_high_pos = 171;
    g9_low_pos = 14;  
  }
  else{
    g9_high_pos = 132;
    g9_low_pos = 53;
  }
  
  // Sweep "Radar" Servo
  if (g9_sweep_up){
    if (g9_servo_pos < g9_high_pos){
      g9_servo_pos++;
      g9_servo.write(g9_servo_pos);  
    }
    else{
      g9_sweep_up = false;  
    }
  }
  if (!g9_sweep_up){
    if (g9_servo_pos > g9_low_pos){
      g9_servo_pos--;
      g9_servo.write(g9_servo_pos);  
    }
    else{
      g9_sweep_up = true;
    }
  }
}

void printObstacleData(){
  /*
   * Prints space seperated X and Y values for obstacle arrays,
   * to be visualized in Processing.
   */
  if (g9_servo_pos == g9_high_pos or (g9_servo_pos == g9_low_pos and g9_sweep_up)){
    for (int i = 0; i < 156; ++i){
      Serial.print(x_coords[i]);
      Serial.print(" ");
    }
    Serial.print("S");
    
    for (int i = 0; i < 156; ++i){
      Serial.print(y_coords[i]);
      Serial.print(" ");
    }
    Serial.println();
  }
}

void steerUpdate(){
  /*
   * Updates the steering angle of the steering servo, by
   * adjusting to correct deviations of the car's heading from
   * 0.0 deg.
   */
  if (abs(car_heading) <= 2.0){
    steer(0);
    digitalWrite(blueLEDPin, LOW);
  }
  else{
    steer_value = -car_heading / 45.0;
    
    if (motor_throttle > 1510){
      steer_value *= -1;  // Reverse steer when car is reversing  
    }

    // Sets a minimum steer amount
    if (abs(steer_value) < 0.25){
      steer_value = (steer_value / abs(steer_value)) * 0.25;  
    }
    steer(steer_value);
    digitalWrite(blueLEDPin, HIGH);
  }
}

void printToLCD(){
  /*
   * Prints data to LCD for debugging.
   */
  lcd.setCursor(0, 0);
  lcd.print("X-Pos: ");
  lcd.print(car_x_pos);

  lcd.setCursor(0, 1);
  lcd.print("Throttle: ");
  lcd.print(motor_throttle);
}

void addPoint(int resolution, int servo_pos){
    /*
     * Adds point to obstacles arrays, using distance measured
     * by time-of-flight sensor.
     * @param resolution: integer representing the frequency
     * by which a point is added, based on current "index" 
     * within the obstacle arrays.
     * @param servo_pos: integer representing sweeping servo
     * position.
     */
    if (coords_index % resolution == 0){
      double theta = map(servo_pos, 14, 171, 0, 180);
      theta = servo_pos * PI / 180.0;
      int radius = readToFSensor();

      x_coords[coords_index / resolution] = int(radius * cos(theta));
      y_coords[coords_index / resolution] = int(radius * sin(theta));
    }
    coords_index++;
    coords_index = coords_index % (156 * resolution);
}

void establishProcessingContact(){
  /*
   * Serial protocol for connecting with Processing.
   */
  while(Serial.available() <= 0){
    Serial.println("A");
    delay(300);  
  }  
}

void setup() {
  Serial.begin(9600);
  while (!Serial){}
  Wire.begin();

  // IMU
  setupIMU();
  callibrateAccelerometer();

  // ToF Sensor
  setupToF();

  // 20kg Servo
  steer_servo.attach(steerServoPin);
  moveSteerServo(STEER_SERVO_MID);  // Straighten front wheels

  // g9 Servo
  g9_servo.attach(g9ServoPin);
  g9_servo.write(86); // Straighten 9g servo

  // ESC aka DC Motor
  ESC.attach(ESCPin);
  setupESC();

  // Ultrasonic Sensor
  pinMode(sonicTrig, OUTPUT);
  pinMode(sonicEcho, INPUT);

  // LEDs
  pinMode(redLEDPin, OUTPUT);
  pinMode(blueLEDPin, OUTPUT);

  // LCD
  lcd.begin(16, 2);

  //establishProcessingContact();

  delay(1000);

  time_ms = millis();
}

void loop() {
  updateOdometry(int(millis() - time_ms));
  time_ms = millis();

  servoSweepStep();
  
  addPoint(2, g9_servo_pos);
  
  if (car_state == OBSTACLE_STOP){
    digitalWrite(redLEDPin, HIGH);
  }
  else{
    digitalWrite(redLEDPin, LOW);
  }
  
  steerUpdate();

  // Updates throttle every 2 sweep servo steps
  if (g9_servo_pos % 2 == 0){
    if (car_state == FORWARD){
      updateForwardThrottle();
    }
    else if (car_state == OBSTACLE_STOP){
      updateReverseThrottle();
    }
  }

  printToLCD();
 
  adjustThrottle(desired_throttle, 3);
}
