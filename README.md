# Arduino Autonomous Car Project

### Project Objective

To design, build, and program an RC car-styled robot which can navigate to any set of coordinates whilst avoiding obstacles.

### Introduction

The goal with this project was to create an autonomously traversing robot from the ground up. This included:
  - Designing a simple chassis & drivetrain
  - Determining the minimal hardware necessary for the desired behavior
  - Writing a program which integrates sensor inputs to control the robot

Powered by an Arduino Mega microprocessor, the robot currently relies on two sensors as inputs. 

The first is a Time-of-Flight (ToF) sensor, which is capable of accurately measuring the distance to the nearest obstacle 
within sight (up to ~1.2 meters away). Mounted to a 9g servo motor, the ToF sensor oscillates roughly 120 degrees, detecting 
any obstacles that may be in the robots path.

The second is an Inertial Measurement Unit (IMU), which is used for taking measurements of the robot's linear acceleration
and angular velocity in the X, Y and Z axis.

For movement, the robot uses a 12v DC motor for driving forward and reversing, as well as a larger servo motor for steering
the front wheels.

![IMG_1105](https://user-images.githubusercontent.com/63004334/131267756-cafef62c-75af-483a-8575-36398ed931fd.JPG)

### Milestone 1

https://user-images.githubusercontent.com/63004334/131267734-36d6d62d-99a6-4d6e-97c9-5c637e625004.mp4

The robot is able to maintain its heading traveling in a straight line—stopping and waiting upon encountering an obstacle. 
Movement is resumed when the obstacle is removed.

### Milestone 2

https://user-images.githubusercontent.com/63004334/131267668-aa9b7302-38d9-43cc-9d87-d7bb65c9efd6.mp4

The robot continuously traverses a room by:
  1. Traveling straight until an obstacle is encountered
  2. Reversing a preconfigured distance
  3. Conducting a left turn (and returning to step 1)

### Technologies
  - Arduino Mega 2560
  - Arduino 1.8.15
  - Processing 4.0b1 (Java)

### Sources

The following are websites from which I drew inspiration or learned how to use various Arduino libraries:
