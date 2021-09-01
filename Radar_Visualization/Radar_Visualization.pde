import processing.serial.*;
Serial myPort;
String vals[] = new String[2];
String serialInput;
String xVals;
String yVals;

int xOffset = 600;
int yOffset = 800;
int xCoords[] = new int[156];
int yCoords[] = new int[156];
String sXCoords[] = new String[156];
String sYCoords[] = new String[156];

boolean firstContact = false;

void setup(){
  size(1200, 800);
  background(255);
  String portName = Serial.list()[0];
  myPort = new Serial(this, portName, 9600);
  myPort.bufferUntil('\n');
}

void draw(){
  background(255);
  draw_box();
  visualizePoints(xCoords, yCoords);
  serialEvent(myPort);
}

void visualizePoints(int x[], int y[]){
  fill(255, 0, 0);
  stroke(255, 0, 0);
  for (int i = 0; i < x.length; ++i){
    ellipse(x[i] + xOffset, yOffset - y[i], 5, 5);
  }
  
  serialEvent(myPort);
}

void draw_box(){
  fill(0, 0, 255);
  stroke(0, 0, 255);
  strokeWeight(2);
  line(xOffset - 210, yOffset, xOffset - 210, yOffset - 340);
  line(xOffset + 210, yOffset, xOffset + 210, yOffset - 340);
  line(xOffset - 210, yOffset - 340, xOffset + 210, yOffset - 340);
}

void serialEvent(Serial myPort){
  serialInput = myPort.readStringUntil('\n');
  
  if (serialInput != null){
    
    if (firstContact == false){
      serialInput = trim(serialInput);
      if (serialInput.equals("A")){
        myPort.clear();
        firstContact = true;
        myPort.write("A");
        println("Contact successful");
      }
    }
    else {
      vals = serialInput.split("S");
      
      xVals = vals[0];
      yVals = vals[1];
      
      xVals = xVals.substring(0, xVals.length() - 1);
      yVals = yVals.substring(0, yVals.length() - 2);
      
      sXCoords = xVals.split(" ");
      sYCoords = yVals.split(" ");
      
      for (int i = 0; i < sXCoords.length; ++i){
        xCoords[i] = Integer.parseInt(sXCoords[i]);
      }
      for (int i = 0; i < sYCoords.length; ++i){
        yCoords[i] = Integer.parseInt(sYCoords[i]);
      }
      
      if (xVals.length() > 10){
        visualizePoints(xCoords, yCoords);
      }
      
      myPort.write("A");
    }
  }
}
