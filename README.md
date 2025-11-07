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
