# Arduino Autonomous Car Project

### Project Objective

To design, build, and program an RC car-styled robot which can navigate to any set of coordinates whilst avoiding obstacles.

### Introduction

The goal with this project was to create an autonomously traversing robot from the ground up. This included:
  - Designing a simple chassis & drivetrain
  - Determining the minimal hardware necessary for the desired behavior
  - Writing a program which integrates sensor inputs and commands the robot's actions

Powered by an Arduino Mega microprocessor, the robot currently relies on two sensors as input. 

The first is a Time-of-Flight (ToF) sensor, which is capable of accurately measuring the distance to the nearest obstacle 
within sight (up to ~1.2 meters away). Mounted to a 9g servo motor, the ToF sensor oscillates roughly 120 degrees, detecting 
any obstacles that may be in the robots path.

The second is an Inertial Measurement Unit (IMU), which is used for taking measurements of the robot's linear acceleration
and angular velocity in the X, Y and Z axis.

For movement, the robot uses a 12v DC motor for driving forward and reversing, as well as a larger servo motor for steering
the front wheels.

### Milestone 1

### Milestone 2

### Technologies

### Sources

The following are websites from which I drew inspiration or learned how to use various Arduino libraries:
