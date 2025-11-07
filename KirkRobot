#include <Wire.h> 
#include "Arduino_LED_Matrix.h"
#include <IRremote.h>
#include <Servo.h>
#include "animation.h"
#include <LiquidCrystal_I2C.h>   // Changed to I2C library
#include <WiFiS3.h>
#include "ArduinoBLE.h"
//#include <WebServer.h>
#include <SoftwareSerial.h>

const char* ssid = "Pornhub";     // Replace with your WiFi name
const char* password = "MyGrace1999!";
// HC-05 TX -> Arduino RX (Pin 2)
// HC-05 RX -> Arduino TX (Pin 3)
SoftwareSerial bluetooth(2, 3); // RX, TX

WiFiServer server(80);

//html code
// HTML page stored in program memory
const char MAIN_page[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
    <title>KIRK Robot Control</title>
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .control-panel {
            max-width: 400px;
            margin: 0 auto;
            text-align: center;
            background-color: white;
            padding: 20px;
            border-radius: 10px;
            box-shadow: 0 2px 5px rgba(0,0,0,0.1);
        }
        .button {
            width: 80px;
            height: 80px;
            margin: 5px;
            font-size: 24px;
            border: none;
            border-radius: 10px;
            background-color: #4CAF50;
            color: white;
            cursor: pointer;
            transition: background-color 0.3s;
        }
        .button:active {
            background-color: #45a049;
        }
        .stop {
            background-color: #f44336;
        }
        .mode {
            background-color: #2196F3;
            width: 120px;
        }
        #status {
            margin-top: 20px;
            padding: 10px;
            background-color: #ddd;
            border-radius: 5px;
        }
    </style>
</head>
<body>
    <div class="control-panel">
        <h1>KIRK Robot Control</h1>
        <div>
            <button class="button" onmousedown="sendCommand('F')" onmouseup="sendCommand('S')">↑</button>
        </div>
        <div>
            <button class="button" onmousedown="sendCommand('L')" onmouseup="sendCommand('S')">←</button>
            <button class="button stop" onclick="sendCommand('S')">■</button>
            <button class="button" onmousedown="sendCommand('R')" onmouseup="sendCommand('S')">→</button>
        </div>
        <div>
            <button class="button" onmousedown="sendCommand('B')" onmouseup="sendCommand('S')">↓</button>
        </div>
        <div>
            <button class="button mode" onclick="sendCommand('A')">Avoid Mode</button>
            <button class="button mode" onclick="sendCommand('T')">Trick</button>
        </div>
        <div id="status">Status: Ready</div>
    </div>

    <script>
        function sendCommand(cmd) {
            fetch('/control?cmd=' + cmd)
                .then(response => response.text())
                .then(data => {
                    document.getElementById('status').innerHTML = 'Status: ' + cmd;
                })
                .catch(error => {
                    console.error('Error:', error);
                    document.getElementById('status').innerHTML = 'Error: ' + error;
                });
        }

        // Update status every second
        setInterval(function() {
            fetch('/status')
                .then(response => response.json())
                .then(data => {
                    document.getElementById('status').innerHTML = 
                        'Status: Mode=' + data.mode + 
                        ', Distance=' + data.distance + 'cm';
                })
                .catch(error => console.error('Error:', error));
        }, 1000);
    </script>
</body>
</html>
)=====";
///////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////

// Bluetooth commands
#define BT_FORWARD  'F'
#define BT_BACKWARD 'B'
#define BT_LEFT     'L'
#define BT_RIGHT    'R'
#define BT_STOP     'S'
#define BT_AVOID    'A'
#define BT_TRICK    'T'

// Define IR Remote codes
#define IR_UP    0xFF629D
#define IR_DOWN  0xFFA857
#define IR_LEFT  0xFF22DD
#define IR_RIGHT 0xFFC23D
#define IR_OK    0xFF02FD

// MODES
#define MODE_STANDBY 0
#define MODE_AVOID   1
#define MODE_MANUAL  2

// Pin Definitions
const int RECV_PIN = 3;    // IR receiver pin
const int left_ctrl = 4;    // Left motor control
const int left_pwm = 5;     // Left motor speed
const int right_ctrl = 7;   // Right motor control
const int right_pwm = 6;    // Right motor speed
const int servopin = 9;     // Servo pin for ultrasonic sensor
const int TRIG_PIN = 12;    // Ultrasonic sensor trigger pin
const int ECHO_PIN = 13;    // Ultrasonic sensor echo pin
const int KEY_LED = 10;
int currentMode = MODE_STANDBY;
bool avoidModeActive = false;

// Constants
const int MOTOR_SPEED = 150;    // Default motor speed (0-255)
const int TURN_SPEED = 120;     // Speed for turning
const int STOP_SPEED = 0;       // Speed for stopping
const int SAFE_DISTANCE = 20;   // Safe distance in centimeters
const int SCAN_DELAY = 500;     // Delay between distance readings

// Add these at the top with your other constants
const bool MOTOR_FORWARD = HIGH;  // Adjust HIGH/LOW based on your motor driver
const bool MOTOR_BACKWARD = LOW;  // Adjust H

// Initialize objects
IRrecv irrecv(RECV_PIN);
decode_results results;
Servo myservo;
ArduinoLEDMatrix matrix;
LiquidCrystal_I2C lcd(0x27, 16, 2);  // Changed to I2C (use 0x3F if 0x27 doesn't work)

// Flag for test sequence
bool testSequenceComplete = false;


// BLE Service and Characteristic definitions
BLEService robotService("180A");  // Robot Control Service
BLECharacteristic controlCharacteristic("2A57", BLERead | BLEWrite, "command");

// Function declarations
void stopMoving();
void gradualStop(int delay_ms = 50);
void updateLCD();


// Function to measure distance using ultrasonic sensor
int getDistance() {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    
    long duration = pulseIn(ECHO_PIN, HIGH);
    int distance = duration * 0.034 / 2;
    
    return distance;
}
// Function to scan surroundings
void scanSurroundings() {
    int distances[3];
    
    myservo.write(0);
    delay(SCAN_DELAY);
    distances[0] = getDistance();
    
    myservo.write(90);
    delay(SCAN_DELAY);
    distances[1] = getDistance();
    
    myservo.write(180);
    delay(SCAN_DELAY);
    distances[2] = getDistance();
    
    myservo.write(90);
    
    Serial.print("Left: ");
    Serial.print(distances[0]);
    Serial.print("cm, Center: ");
    Serial.print(distances[1]);
    Serial.print("cm, Right: ");
    Serial.print(distances[2]);
    Serial.println("cm");
    
    for(int i = 0; i < 3; i++) {
        if(distances[i] < SAFE_DISTANCE && distances[i] > 0) {
            gradualStop(50);
            Serial.println("Obstacle detected! Stopping!");
            break;
        }
    }
}

// Function to check distance at specific angle
int checkDistance(int angle) {
    myservo.write(angle);
    delay(500);
    int distance = getDistance();
    Serial.print("Distance at angle ");
    Serial.print(angle);
    Serial.print(": ");
    Serial.println(distance);
    return distance;
}

// LCD update function
void updateLCD() {
    lcd.clear();
    lcd.setCursor(0, 0);
    
    // First line: Mode display
    switch(currentMode) {
        case MODE_STANDBY:
            lcd.print("Mode: Standby");
            break;
        case MODE_AVOID:
            lcd.print("Mode: Avoid");
            break;
        case MODE_MANUAL:
            lcd.print("Mode: Manual");
            break;
    }
    
    // Second line: Distance reading
    lcd.setCursor(0, 1);
    int distance = getDistance();
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print("cm");
}

// Obstacle avoidance behavior
void avoidanceRoutine() {
    int frontDistance = getDistance();
    
    // Debug output
    Serial.print("Front Distance: ");
    Serial.println(frontDistance);
    
    // First, check if path is clear - if so, just keep moving forward
    if(frontDistance > SAFE_DISTANCE) {
        moveForward();
        return;
    }
    
    // If we get here, we've found an obstacle - stop first
    gradualStop(50);
    
    // Comprehensive scan of surroundings
    myservo.write(0);    // Look left
    delay(300);
    int leftDistance = getDistance();
    
    myservo.write(90);   // Look center
    delay(300);
    frontDistance = getDistance();  // Take a fresh center reading
    
    myservo.write(180);  // Look right
    delay(300);
    int rightDistance = getDistance();
    
    // Return servo to forward position
    myservo.write(90);
    
    // Debug output
    Serial.println("Distances measured:");
    Serial.print("Left: "); Serial.print(leftDistance);
    Serial.print(" cm, Front: "); Serial.print(frontDistance);
    Serial.print(" cm, Right: "); Serial.print(rightDistance);
    Serial.println(" cm");
    
    // Decision making with improved logic
    if(frontDistance > SAFE_DISTANCE) {
        // Path is clear, move forward
        moveForward();
    }
    else if(leftDistance > SAFE_DISTANCE && leftDistance > rightDistance) {
        // Left is clear and better than right
        Serial.println("Turning left");
        turnLeft();
        delay(500);  // Shorter turn duration
        stopMoving();
    }
    else if(rightDistance > SAFE_DISTANCE && rightDistance > leftDistance) {
        // Right is clear and better than left
        Serial.println("Turning right");
        turnRight();
        delay(500);  // Shorter turn duration
        stopMoving();
    }
    else {
        // All directions blocked or unclear - make a larger turn
        Serial.println("All directions blocked - backing up and turning");
        // Back up first
        moveBackward();
        delay(1000);
        stopMoving();
        
        // Make a larger turn (choose direction based on which side has more space)
        if(leftDistance > rightDistance) {
            turnLeft();
        } else {
            turnRight();
        }
        delay(1000);
        stopMoving();the keye
    
    // Small delay before next scan
    delay(100);
}

// Movement functions
void moveForward() {
    if(getDistance() > SAFE_DISTANCE) {
        // Make sure these HIGH/LOW settings match your motor driver configuration
        digitalWrite(left_ctrl, HIGH);   // Check if HIGH is actually forward
        digitalWrite(right_ctrl, HIGH);  // Check if HIGH is actually forward
        analogWrite(left_pwm, MOTOR_SPEED);
        analogWrite(right_pwm, MOTOR_SPEED);
        
        // Debug output
        Serial.println("Moving Forward");
        Serial.print("Distance: ");
        Serial.println(getDistance());
    } else {
        gradualStop(50);
        Serial.println("Obstacle ahead! Stopping!");
    }
}


void moveBackward() {
    digitalWrite(left_ctrl, LOW);
    digitalWrite(right_ctrl, LOW);
    analogWrite(left_pwm, MOTOR_SPEED);
    analogWrite(right_pwm, MOTOR_SPEED);
}

void turnLeft() {
    digitalWrite(left_ctrl, LOW);
    digitalWrite(right_ctrl, HIGH);
    analogWrite(left_pwm, TURN_SPEED);
    analogWrite(right_pwm, TURN_SPEED);
}

void turnRight() {
    digitalWrite(left_ctrl, HIGH);
    digitalWrite(right_ctrl, LOW);
    analogWrite(left_pwm, TURN_SPEED);
    analogWrite(right_pwm, TURN_SPEED);
}

void stopMoving() {
    analogWrite(left_pwm, STOP_SPEED);
    analogWrite(right_pwm, STOP_SPEED);
}

void gradualStop(int delay_ms) {
    for(int speed = MOTOR_SPEED; speed >= 0; speed -= 10) {
        analogWrite(left_pwm, speed);
        analogWrite(right_pwm, speed);
        delay(delay_ms);
    }
    stopMoving();
}
void testMotorDirections() {
    Serial.println("Testing motor directions...");
    
    // Test left motor
    Serial.println("Testing left motor forward...");
    digitalWrite(left_ctrl, HIGH);
    analogWrite(left_pwm, MOTOR_SPEED);
    delay(2000);
    stopMoving();
    delay(1000);
    
    // Test right motor
    Serial.println("Testing right motor forward...");
    digitalWrite(right_ctrl, HIGH);
    analogWrite(right_pwm, MOTOR_SPEED);
    delay(2000);
    stopMoving();
    
    Serial.println("Motor test complete");
}

// Trick performance
void performTrick() {
    Serial.println("Performing trick!");
    
    turnRight();
    delay(3000);
    stopMoving();
    delay(500);
    
    for(int i = 0; i < 2; i++) {
        myservo.write(0);
        delay(500);
        myservo.write(180);
        delay(500);
    }
    
    myservo.write(90);
    stopMoving();
    currentMode = MODE_STANDBY;
}

void runTestSequence() {
    Serial.println("Testing Servo and Ultrasonic Sensor...");
    scanSurroundings();
    delay(1000);
    
    moveForward();
    delay(2000);
    stopMoving();
    delay(1000);
    
    turnLeft();
    delay(1000);
    stopMoving();
    delay(1000);
    
    turnRight();
    delay(1000);
    stopMoving();
    delay(1000);
    
    moveBackward();
    delay(2000);
    gradualStop(50);
    delay(2000);
    
    testSequenceComplete = true;
}

void setup() {
    Serial.begin(9600);
     // Start Bluetooth communication
  bluetooth.begin(9600);  // HC-05 default baud rate
    testMotorDirections();
    // Initialize WiFi
    WiFi.begin(ssid, password);
    
    // Wait for WiFi connection
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println("");
    Serial.println("WiFi connected");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
      // Print the IP address clearly
    Serial.println("\n----------------------------------------");
    Serial.println("Connected to WiFi!");
    Serial.print("Arduino's IP address: ");
    Serial.println(WiFi.localIP());
    Serial.println("----------------------------------------");

    // Start web server
    server.begin();
    
    // Your existing setup code continues here
    pinMode(left_ctrl, OUTPUT);
    pinMode(left_pwm, OUTPUT);
    pinMode(right_ctrl, OUTPUT);
    pinMode(right_pwm, OUTPUT);
    pinMode(TRIG_PIN, OUTPUT);
    pinMode(ECHO_PIN, INPUT);
    pinMode(KEY_LED, OUTPUT);
    
    // Initialize LCD
    Wire.begin();
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.print("KIRK Robot v1.0");
    
    // Display WiFi info on LCD
    lcd.setCursor(0, 1);
    lcd.print(WiFi.localIP());
    
    delay(200);
    
    // Initialize other components
    irrecv.enableIRIn();
    myservo.attach(servopin);
    myservo.write(90);
    
    matrix.begin();
    matrix.loadSequence(frames);
    matrix.play(true);
    
    Serial.println("KIRK ROBOT Ready!");
}

// Step 5: Modify the loop() function to handle web clients
void loop() {
    // Handle web clients
    WiFiClient client = server.available();
    if (client) {
        String currentLine = "";
        while (client.connected()) {
            if (client.available()) {
                char c = client.read();
                if (c == '\n') {
                    if (currentLine.length() == 0) {
                        // HTTP headers
                        client.println("HTTP/1.1 200 OK");
                        client.println("Content-type:text/html");
                        client.println();
                        
                        // Web page
                        client.print(MAIN_page);
                        break;
                    } else {
                        // Check for commands
                        if (currentLine.indexOf("GET /control?cmd=") >= 0) {
                            char cmd = currentLine.charAt(currentLine.indexOf("cmd=") + 4);
                            handleCommand(cmd);
                        }
                        currentLine = "";
                    }
                } else if (c != '\r') {
                    currentLine += c;
                }
            }
        }
        client.stop();
    }

    // Your existing loop code continues here
    updateLCD();
    delay(200);

    if (bluetooth.available()) {
        String message = bluetooth.readString();
        // Display on LCD
        lcd.clear();
        lcd.print(message.substring(0, 16)); // First line (16 chars)
        if(message.length() > 16) {
            lcd.setCursor(0, 1);
            lcd.print(message.substring(16, 32)); // Second line (next 16 chars)
        }
        // Also print to Serial Monitor
        Serial.println("Received: " + message);
    }

    if(Serial.available() > 0) {
        char command = Serial.read();
        handleCommand(command);
    }
    
    if (irrecv.decode(&results)) {
        // Your existing IR control code...
        irrecv.resume();
        updateLCD();
    }

    switch(currentMode) {
        case MODE_AVOID:
            avoidanceRoutine();
            break;
        case MODE_STANDBY:
            stopMoving();
            break;
        case MODE_MANUAL:
            break;
    }

    if (Serial.available()) {
        String message = Serial.readString();
        bluetooth.println(message);  // Send to HC-05
    }
} //

void handleCommand(char command) {
    lcd.clear();
    lcd.setCursor(0, 0);
    
    switch(command) {
        case BT_FORWARD:
        case 'f':
            currentMode = MODE_MANUAL;
            moveForward();
            lcd.print("Moving Forward");
            break;
        case BT_BACKWARD:
        case 'b':
            currentMode = MODE_MANUAL;
            moveBackward();
            lcd.print("Moving Backward");
            break;
        case BT_LEFT:
        case 'l':
            currentMode = MODE_MANUAL;
            turnLeft();
            lcd.print("Turning Left");
            break;
        case BT_RIGHT:
        case 'r':
            currentMode = MODE_MANUAL;
            turnRight();
            lcd.print("Turning Right");
            break;
        case BT_STOP:
        case 's':
            currentMode = MODE_STANDBY;
            gradualStop(50);
            lcd.print("Stopped");
            break;
        case BT_AVOID:
        case 'a':
            currentMode = MODE_AVOID;
            lcd.print("Avoid Mode");
            break;
        case BT_TRICK:
        case 't':
            performTrick();
            lcd.print("Performing Trick");
            break;
    }
    
    lcd.setCursor(0, 1);
    int distance = getDistance();
    lcd.print("Dist: ");
    lcd.print(distance);
    lcd.print("cm");
}


