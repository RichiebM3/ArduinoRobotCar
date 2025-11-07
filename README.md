# ArduinoRobotCar
Arduino, Ultrasonic sensor, motor driver, 12V battery and an ultasonic sensor paired with a servo motor for mount detections.

# KIRK Robot Control

## Overview
KIRK Robot Control is an Arduino-based project that allows you to control a robot using various input methods, including Bluetooth, IR remote, and a web interface. The robot is equipped with ultrasonic sensors for obstacle avoidance and can perform tricks. The project utilizes various libraries such as Servo, LiquidCrystal_I2C, and WiFi to facilitate communication and control.

## Table of Contents
- [Features](#features)
- [Getting Started](#getting-started)
- [Key Components](#key-components)
- [Functionality Overview](#functionality-overview)
- [Controls](#controls)
- [Learning Objectives](#learning-objectives)
- [Bug Fixes and Improvements](#bug-fixes-and-improvements)

## Features
- Control the robot via Bluetooth, IR remote, or web interface.
- Obstacle avoidance using ultrasonic sensors.
- LCD display for real-time status updates.
- Servo-controlled scanning for distance measurement.
- Perform predefined tricks.

## Getting Started
To run the KIRK Robot Control project, follow these steps:

1. **Hardware Requirements**:
   - Arduino board (e.g., Arduino Uno)
   - HC-05 Bluetooth module
   - IR remote and receiver
   - Servo motor (for ultrasonic sensor scanning)
   - Ultrasonic distance sensor (HC-SR04)
   - Motor driver module (for controlling motors)
   - 16x2 I2C LCD display
   - Other necessary components (wires, breadboard, etc.)

2. **Library Installation**:
   - Install the required libraries via the Arduino Library Manager:
     - `IRremote`
     - `Servo`
     - `LiquidCrystal_I2C`
     - `WiFiS3`
     - `ArduinoBLE`
     - `Arduino_LED_Matrix`

3. **Setup**:
   - Connect the components according to the pin definitions in the code.
   - Update the WiFi credentials in the code:
     ```cpp
     const char* ssid = "Your_SSID";     // Replace with your WiFi name
     const char* password = "Your_Password"; // Replace with your WiFi password
     ```

4. **Upload the Code**:
   - Upload the provided code to your Arduino board using the Arduino IDE.

## Key Components
- **WiFi**: Allows the robot to connect to a local network for web control.
- **Bluetooth**: Enables control via Bluetooth commands from a smartphone or other devices.
- **IR Remote**: Allows control using an infrared remote.
- **Ultrasonic Sensor**: Measures distance to detect obstacles.
- **LCD Display**: Shows the current mode and distance readings.

## Functionality Overview

### Setup Function
- Initializes the serial communication, Bluetooth, WiFi, and various components.
- Displays the IP address on the LCD after connecting to WiFi.

### Loop Function
- Handles incoming web requests and Bluetooth commands.
- Updates the LCD with distance readings and current mode.
- Executes movement commands based on the current mode (manual, avoid).

### Movement Functions
- `moveForward()`, `moveBackward()`, `turnLeft()`, `turnRight()`: Control the robot's movement.
- `stopMoving()`, `gradualStop()`: Stop the robot smoothly.

### Obstacle Avoidance
- `scanSurroundings()`: Scans the environment for obstacles and takes action based on distance readings.
- `avoidanceRoutine()`: Implements logic to navigate around obstacles.

### Command Handling
- `handleCommand(char command)`: Processes commands received via Bluetooth, IR, or web interface.

### LCD Update
- `updateLCD()`: Refreshes the LCD display with the current mode and distance reading.

## Controls
- **Bluetooth Commands**:
  - **F**: Move Forward
  - **B**: Move Backward
  - **L**: Turn Left
  - **R**: Turn Right
  - **S**: Stop
  - **A**: Activate Avoid Mode
  - **T**: Perform Trick

- **IR Remote**: Corresponding buttons on the remote can be mapped to the same commands.

- **Web Interface**: Control the robot using the web page served by the robot.

## Learning Objectives
By working on this project, you will learn:
- How to interface various sensors and actuators with Arduino.
- Techniques for handling multiple communication protocols (Bluetooth, WiFi, IR).
- Implementing real-time control systems and obstacle avoidance algorithms.
- Using libraries effectively for hardware control.

## Bug Fixes and Improvements
### Identified Bugs and Corrections
1. **WiFi Connection Handling**: Improved the handling of WiFi connection status to ensure robust connectivity.
2. **Servo Positioning**: Fixed the servo positioning logic to ensure accurate scanning of surroundings.
3. **LCD Display Updates**: Ensured the LCD updates correctly during movement and command handling.
4. **Motor Control Logic**: Adjusted motor control logic to ensure correct HIGH/LOW settings based on the motor driver configuration.
5. **Command Handling**: Enhanced command handling to avoid potential issues with unexpected characters.
6. **Distance Measurement**: Included checks to ensure valid distance readings before making movement decisions.

### Suggested Improvements
- Implement error handling for Bluetooth and WiFi connections.
- Add more advanced obstacle avoidance algorithms.
- Enhance the web interface with additional status indicators.
- Optimize the code for better performance and readability.

## Conclusion
The KIRK Robot Control project provides a comprehensive platform for learning about robotics and Arduino programming. By exploring and modifying the code, you can gain valuable insights into hardware control, communication protocols, and real-time decision-making in robotics.

Feel free to experiment and expand the functionalities of the KIRK robot!

Revision verion changes below, above is focused on 1.0 version

Explanation of the Code
Libraries and Header Files
Wire.h: For I2C communication.
Arduino_LED_Matrix.h: Library for controlling LED matrices.
IRremote.h: Library to control the IR remote.
Servo.h: Library for controlling servo motors.
LiquidCrystal_I2C.h: Library for controlling LCDs via I2C.
WiFiS3.h: Library for WiFi connectivity.
ArduinoBLE.h: Library for Bluetooth Low Energy (BLE) communication.
SoftwareSerial.h: Library to create a serial interface on other digital pins.
Global Variables and Constants
WiFi Credentials: ssid and password for connecting to a WiFi network.
Bluetooth Commands: Constants for different movement commands (forward, backward, left, right, stop, avoid, trick).
IR Remote Codes: Constants for IR commands corresponding to specific buttons on the remote.
Modes: Constants to define the robot's operational modes (standby, avoid, manual).
Pin Definitions: Constants for pin assignments for various components (motors, ultrasonic sensors, etc.).
Motor Control Constants: Constants for motor speeds and safe distance thresholds.
Setup Function
Initializes serial communication, Bluetooth, and WiFi.
Connects to the specified WiFi network and prints the IP address.
Initializes pins for motors, ultrasonic sensors, and the LCD.
Sets up the IR receiver and servo motor.
Loop Function
Handles incoming web client requests and processes commands.
Updates the LCD display with the current distance and mode.
Checks for Bluetooth and IR commands and executes the corresponding actions.
Executes behavior based on the current operational mode.
Movement Functions
moveForward(): Moves the robot forward if no obstacles are detected.
moveBackward(): Moves the robot backward.
turnLeft(): Turns the robot left.
turnRight(): Turns the robot right.
stopMoving(): Stops the robot immediately.
gradualStop(): Gradually reduces speed to a stop.
Obstacle Avoidance
getDistance(): Measures distance using the ultrasonic sensor.
scanSurroundings(): Scans the environment for obstacles by checking distances at different angles.
avoidanceRoutine(): Implements obstacle avoidance logic based on distance readings.
Command Handling
handleCommand(): Processes commands received from Bluetooth, IR, or web interface and updates the robot's mode and actions.
performTrick(): Executes a predefined trick involving turning and moving the servo.
Summary of Improvements
Command Handling: Improved handling of Bluetooth and IR commands to ensure they are processed correctly.
Distance Measurement: Enhanced the distance measurement logic to ensure accurate readings.
LCD Updates: Streamlined the LCD updates to reflect the current state and distance more effectively.
Modular Functions: Organized the code into clear, modular functions for better readability and maintenance.

