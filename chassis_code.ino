/*
  * Authors:
  * Nathan Gordon
  * Tiffany Campbell
*/

#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <IRremote.hpp>

// ----- Motor Driver Pins (TB6612) -----
#define PIN_Motor_PWMA 5
#define PIN_Motor_PWMB 6
#define PIN_Motor_AIN1 7
#define PIN_Motor_BIN1 8
#define PIN_Motor_STBY 3

// ----- Ultrasonic Sensor Pins -----
#define ULTRA_TRIG_PIN 13
#define ULTRA_ECHO_PIN 12

// ----- IR Receiver Pin -----
#define IR_RECEIVE_PIN 9

// ----- Operating Modes -----
#define MODE_FORWARD        1
#define MODE_REVERSE        2
#define MODE_TURN_LEFT      3
#define MODE_TURN_RIGHT     4
#define MODE_TURN_AROUND    5
#define MODE_STOP           6
#define MODE_CHANGE_DISTANCE 7

// ----- Motor Speed Settings -----
#define MOTOR_SPEED_NORMAL  180    // Normal speed (0-255)
#define MOTOR_SPEED_TURN    150    // Turning speed (0-255)
#define MOTOR_SPEED_SLOW    120    // Slow speed (0-255)

// ----- Obstacle Avoidance Settings -----
#define OBSTACLE_DISTANCE   20      // Distance in cm to detect obstacles
#define SLOW_DOWN_FACTOR    1.5    // Distance multiplier for when to start slowing down
#define DISTANCE_CLOSE      5
#define DISTANCE_MEDIUM     10
#define DISTANCE_FAR        15

// ----- PID parameters for turning and straight drive -----
float Kp = 2.0;    // Proportional gain
float Ki = 0.4;    // Integral gain
float Kd = 0.5;    // Derivative gain
float errorSum = 0.0;
float lastError = 0.0;
float yaw = 0.0;

// ----- PID parameters for distance control (stopping) -----
float Kp_dist = 2.5;
float Ki_dist = 0.1;
float Kd_dist = 0.8;
float errorSum_dist = 0.0;
float lastError_dist = 0.0;
float lastDistance = 0.0;

// global variables
Adafruit_MPU6050 mpu;
byte currentMode = MODE_STOP;
unsigned long lastCommandTime = 0;
unsigned long lastUpdateTime = 0;
int leftMotorSpeed = 0;
int rightMotorSpeed = 0;
int inputDistance = OBSTACLE_DISTANCE;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize Motor Driver pins:
  pinMode(PIN_Motor_PWMA, OUTPUT);
  pinMode(PIN_Motor_PWMB, OUTPUT);
  pinMode(PIN_Motor_AIN1, OUTPUT);
  pinMode(PIN_Motor_BIN1, OUTPUT);
  pinMode(PIN_Motor_STBY, OUTPUT);
  
  // enable the motor driver:
  digitalWrite(PIN_Motor_STBY, HIGH);
  
  // initialize Ultrasonic Sensor pins:
  pinMode(ULTRA_TRIG_PIN, OUTPUT);
  pinMode(ULTRA_ECHO_PIN, INPUT);
  
  // initialize IR Receiver:
  IrReceiver.begin(IR_RECEIVE_PIN, ENABLE_LED_FEEDBACK);
  
  // initialize MPU6050:
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  
  // configure MPU6050:
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  
  lastUpdateTime = millis();
  
  Serial.println("4WD Car Ready!");
  delay(1000);
}

void loop() {
  // check for IR remote commands:
  if (checkIRCommands()) {
    // if a new command was received, handle it:
    handleCommand();
  }
  
  // continue current operation based on mode:
  switch (currentMode) {
    case MODE_FORWARD:
      moveForwardGyro(MOTOR_SPEED_NORMAL);
      break;
      
    case MODE_REVERSE:
      reverseGyro(MOTOR_SPEED_NORMAL);
      break;
      
    case MODE_TURN_LEFT:
      turnLeft(MOTOR_SPEED_TURN);
      break;
      
    case MODE_TURN_RIGHT:
      turnRight(MOTOR_SPEED_TURN);
      break;
      
    case MODE_TURN_AROUND:
      turnAround(MOTOR_SPEED_TURN);
      break;
      
    case MODE_STOP:
    default:
      motorStop();
      break;
  }
  
  delay(10);  // Short delay for stability
}

bool checkIRCommands() {
  bool newCommand = false;
  
  if (IrReceiver.decode()) {
    uint32_t irCode = IrReceiver.decodedIRData.decodedRawData;
    
    Serial.print("Received IR code: 0x");
    Serial.println(irCode, HEX);
    
    if (irCode) {
      lastCommandTime = millis();
      newCommand = true;
      
      // right turn:
      if (irCode == 0xBB44FF00) { 
        currentMode = MODE_TURN_RIGHT;
        Serial.println("Command: Turn Right");
        
      // move forward:
      } else if (irCode == 0xB946FF00) {
        currentMode = MODE_FORWARD;
        Serial.println("Command: Forward");
        
      // left turn:
      } else if (irCode == 0xBC43FF00) {
        currentMode = MODE_TURN_LEFT;
        Serial.println("Command: Turn Left");
        
      // reverse:
      } else if (irCode == 0xEA15FF00) {
        currentMode = MODE_REVERSE;
        Serial.println("Command: Reverse");
        
      // 180 turn around:
      } else if (irCode == 0xBF40FF00) {
        currentMode = MODE_TURN_AROUND;
        Serial.println("Command: Turn Around");
        
      // stop all movement:
      } else if (irCode == 0xE916FF00) {
        currentMode = MODE_STOP;
        Serial.println("Command: Stop");
      
      // cycle through setpoints (5, 10, 15)
      } else if (irCode == 0xF30CFF00) {
          if (inputDistance == DISTANCE_CLOSE)
            inputDistance = DISTANCE_MEDIUM;
          else if (inputDistance == DISTANCE_MEDIUM)
            inputDistance = DISTANCE_FAR;
          else
            inputDistance = DISTANCE_CLOSE;
            
          Serial.print("Distance set to: ");
          Serial.println(inputDistance);
        }
    }
    
    IrReceiver.resume();    // ready to receive the next command
  }
  
  return newCommand;
}

bool checkForStop() {
  if (IrReceiver.decode()) {
    uint32_t irCode = IrReceiver.decodedIRData.decodedRawData;
    
    Serial.print("Received IR code: 0x");
    Serial.println(irCode, HEX);
    
    IrReceiver.resume();
    
    if (irCode == 0xE916FF00) {
      currentMode = MODE_STOP;
      return true;
    }
  }
  return false;
}

long measureDistance() {
  // send ultrasonic pulse:
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRA_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRA_TRIG_PIN, LOW);
  
  // read the echo:
  long duration = pulseIn(ULTRA_ECHO_PIN, HIGH);
  
  // calculate distance in cm:
  long distance = duration * 0.034 / 2;
  
  return distance;
}

void handleCommand() {
  // reset PID variables when starting a new command:
  yaw = 0.0;
  errorSum = 0.0;
  lastError = 0.0;
  
  // reset distance PID variables:
  errorSum_dist = 0.0;
  lastError_dist = 0.0;
  lastDistance = 0.0;
  
  lastUpdateTime = millis();
}

// calculate motor speed for precise stopping using PID:
int calculateStoppingSpeed(float currentDist, float targetDist, float dt) {
  // calculate the error (difference between target and current):
  float error = currentDist - targetDist;
  
  // only apply PID if we need to slow down (pproaching the target):
  if (error > 0) {
    // calculate integral term with anti-windup:
    errorSum_dist += error * dt;
    errorSum_dist = constrain(errorSum_dist, -50, 50);    // prevent excessive buildup
    
    // calculate derivative term:
    float dError = (error - lastError_dist) / dt;
    
    // PID output calculation:
    float pidOutput = Kp_dist * error + Ki_dist * errorSum_dist + Kd_dist * dError;
    
    // store current error for next iteration:
    lastError_dist = error;
    
    // convert PID output to motor speed (0-255):
    int motorSpeed = constrain((int)pidOutput, 0, 255);
    
    Serial.print("Distance PID: error=");
    Serial.print(error);
    Serial.print(" output=");
    Serial.print(pidOutput);
    Serial.print(" speed=");
    Serial.println(motorSpeed);
    
    return motorSpeed;
  } else {
    // if we're at or past the target, stop
    return 0;
  }
}

void moveForwardGyro(int baseSpeed) {
  yaw = 0.0;
  errorSum = 0.0;
  lastError = 0.0;
  
  // reset distance PID variables:
  errorSum_dist = 0.0;
  lastError_dist = 0.0;
  
  unsigned long localLastUpdate = millis();
  bool approachingObstacle = false;
  
  while (true) {
    unsigned long currentTime = millis();
    float dt = (currentTime - localLastUpdate) / 1000.0;
    localLastUpdate = currentTime;
    
    // get gyroscope data for straight-line stability:
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float deltaYaw = g.gyro.z * dt * (180.0 / PI);
    yaw += deltaYaw;
    
    // PID for straight line movement:
    float error = 0.0 - yaw;
    errorSum += error * dt;
    errorSum = constrain(errorSum, -50, 50);    // prevent excessive buildup
    float dError = (error - lastError) / dt;
    float pidOutput = Kp * error + Ki * errorSum + Kd * dError;
    lastError = error;
    
    // measure distance to potential obstacles:
    long currentDistance = measureDistance();
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.println(" cm");
    
    // check if we need to stop command:
    if (checkForStop()) {
      motorStop();
      Serial.println("STOP command received! Aborting forward.");
      return;
    }
    
    // determine if we're approaching the stop point:
    if (currentDistance > 0 && currentDistance < (inputDistance * SLOW_DOWN_FACTOR)) {
      approachingObstacle = true;
      
      // if very close to target distance, use PID to approach exactly:
      if (currentDistance < inputDistance * 1.2) {
        // calculate speed based on distance PID:
        int approachSpeed = calculateStoppingSpeed(currentDistance, inputDistance, dt);
        
        // apply straight-line correction from gyro:
        int leftSpeed = approachSpeed + (int)pidOutput;
        int rightSpeed = approachSpeed - (int)pidOutput;
        
        // ensure speeds are within valid ranges:
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        
        Serial.print("Approaching target - leftSpeed=");
        Serial.print(leftSpeed);
        Serial.print(" rightSpeed=");
        Serial.println(rightSpeed);
        
        // check if we've reached the target distance:
        if (abs(currentDistance - inputDistance) < 0.5 || approachSpeed <= 0) {
          motorStop();
          Serial.println("Target distance reached! Stopping precisely.");
          break;
        } else {
          driveForward(leftSpeed, rightSpeed);
        }
      } else {
        // gradually slow down as we approach:
        float distanceFactor = (currentDistance - inputDistance) / (inputDistance * (SLOW_DOWN_FACTOR - 1));
        int reducedSpeed = baseSpeed * constrain(distanceFactor, 0.3, 1.0);
        
        // apply straight-line correction:
        int leftSpeed = reducedSpeed + (int)pidOutput;
        int rightSpeed = reducedSpeed - (int)pidOutput;
        
        leftSpeed = constrain(leftSpeed, 50, 255);
        rightSpeed = constrain(rightSpeed, 50, 255);
        
        Serial.print("Slowing down - factor=");
        Serial.print(distanceFactor);
        Serial.print(" leftSpeed=");
        Serial.print(leftSpeed);
        Serial.print(" rightSpeed=");
        Serial.println(rightSpeed);
        
        driveForward(leftSpeed, rightSpeed);
      }
    } 
    // normal operation when no obstacles are nearby:
    else if (!approachingObstacle) {
      int leftSpeed = baseSpeed + (int)pidOutput;
      int rightSpeed = baseSpeed - (int)pidOutput;
      leftSpeed = constrain(leftSpeed, 50, 255);
      rightSpeed = constrain(rightSpeed, 50, 255);
      
      Serial.print("Normal forward: yaw=");
      Serial.print(yaw);
      Serial.print(" error=");
      Serial.print(error);
      Serial.print(" leftSpeed=");
      Serial.print(leftSpeed);
      Serial.print(" rightSpeed=");
      Serial.println(rightSpeed);
      
      driveForward(leftSpeed, rightSpeed);
    }
    
    // check for new IR commands:
    if (checkIRCommands()) {
      // if we got a new command, exit this function:
      if (currentMode != MODE_FORWARD) {
        return;
      }
    }
    
    delay(10);
  }
}

void reverseGyro(int baseSpeed) {
  yaw = 0.0;
  errorSum = 0.0;
  lastError = 0.0;
  
  // reset distance PID variables:
  errorSum_dist = 0.0;
  lastError_dist = 0.0;
  
  unsigned long localLastUpdate = millis();
  bool approachingObstacle = false;
  
  while (true) {
    unsigned long currentTime = millis();
    float dt = (currentTime - localLastUpdate) / 1000.0;
    localLastUpdate = currentTime;
    
    // get gyroscope data for straight-line stability:
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float deltaYaw = -g.gyro.z * dt * (180.0 / PI);
    yaw += deltaYaw;
    
    // PID for straight line movement:
    float error = 0.0 - yaw;
    errorSum += error * dt;
    errorSum = constrain(errorSum, -50, 50);      // prevents excessive buildup
    float dError = (error - lastError) / dt;
    float pidOutput = Kp * error + Ki * errorSum + Kd * dError;
    lastError = error;
    
    // measure distance to potential obstacles:
    long currentDistance = measureDistance();
    Serial.print("Distance: ");
    Serial.print(currentDistance);
    Serial.println(" cm");
    
    // check if we need to stop command:
    if (checkForStop()) {
      motorStop();
      Serial.println("STOP command received! Aborting reverse.");
      return;
    }
    
    // determine if car is approaching the stop point:
    if (currentDistance > 0 && currentDistance < (inputDistance * SLOW_DOWN_FACTOR)) {
      approachingObstacle = true;
      
      // if very close to the target distance, use PID to approach exactly:
      if (currentDistance < inputDistance * 1.2) {
        // calculate speed based on distance PID:
        int approachSpeed = calculateStoppingSpeed(currentDistance, inputDistance, dt);
        
        // apply straight-line correction from gyro:
        int leftSpeed = approachSpeed + (int)pidOutput;
        int rightSpeed = approachSpeed - (int)pidOutput;
        
        // ensure speeds are within valid ranges:
        leftSpeed = constrain(leftSpeed, 0, 255);
        rightSpeed = constrain(rightSpeed, 0, 255);
        
        Serial.print("Approaching target (reverse) - leftSpeed=");
        Serial.print(leftSpeed);
        Serial.print(" rightSpeed=");
        Serial.println(rightSpeed);
        
        // check if target distance has been reached:
        if (abs(currentDistance - inputDistance) < 0.5 || approachSpeed <= 0) {
          motorStop();
          Serial.println("Target distance reached! Stopping precisely.");
          break;
        } else {
          driveReverse(leftSpeed, rightSpeed);
        }
      } else {
        // gradually slow down as approach setpoint:
        float distanceFactor = (currentDistance - inputDistance) / (inputDistance * (SLOW_DOWN_FACTOR - 1));
        int reducedSpeed = baseSpeed * constrain(distanceFactor, 0.3, 1.0);
        
        // apply straight-line correction:
        int leftSpeed = reducedSpeed + (int)pidOutput;
        int rightSpeed = reducedSpeed - (int)pidOutput;
        
        leftSpeed = constrain(leftSpeed, 50, 255);
        rightSpeed = constrain(rightSpeed, 50, 255);
        
        Serial.print("Slowing down (reverse) - factor=");
        Serial.print(distanceFactor);
        Serial.print(" leftSpeed=");
        Serial.print(leftSpeed);
        Serial.print(" rightSpeed=");
        Serial.println(rightSpeed);
        
        driveReverse(leftSpeed, rightSpeed);
      }
    } 
    // normal operation when no obstacles are nearby:
    else if (!approachingObstacle) {
      int leftSpeed = baseSpeed + (int)pidOutput;
      int rightSpeed = baseSpeed - (int)pidOutput;
      leftSpeed = constrain(leftSpeed, 50, 255);
      rightSpeed = constrain(rightSpeed, 50, 255);
      
      Serial.print("Normal reverse: yaw=");
      Serial.print(yaw);
      Serial.print(" error=");
      Serial.print(error);
      Serial.print(" leftSpeed=");
      Serial.print(leftSpeed);
      Serial.print(" rightSpeed=");
      Serial.println(rightSpeed);
      
      driveReverse(leftSpeed, rightSpeed);
    }
    
    // check for new IR commands:
    if (checkIRCommands()) {
      // if new command is received, exit this function:
      if (currentMode != MODE_REVERSE) {
        return;
      }
    }
    
    delay(10);
  }
}

void turnLeft(int speed) {
  driveMotors(speed, speed, false, true);
  
  // check for stop command or new commands:
  if (checkIRCommands()) {
    if (currentMode != MODE_TURN_LEFT) {
      return;
    }
  }
}

void turnRight(int speed) {
  driveMotors(speed, speed, true, false);
  
  // check for stop command or new commands:
  if (checkIRCommands()) {
    if (currentMode != MODE_TURN_RIGHT) {
      return;
    }
  }
}

void turnAround(int speed) {
  driveMotors(speed, speed, false, true);
  
  // check for stop command or new commands:
  if (checkIRCommands()) {
    if (currentMode != MODE_TURN_AROUND) {
      return;
    }
  }
}

void driveForward(int leftSpeed, int rightSpeed) {
  driveMotors(leftSpeed, rightSpeed, true, true);
}

void driveReverse(int leftSpeed, int rightSpeed) {
  driveMotors(leftSpeed, rightSpeed, false, false);
}

void driveMotors(int leftSpeed, int rightSpeed, bool leftForward, bool rightForward) {
  // set motor directions:
  digitalWrite(PIN_Motor_AIN1, leftForward ? HIGH : LOW);
  digitalWrite(PIN_Motor_BIN1, rightForward ? HIGH : LOW);
  
  // set motor speeds:
  analogWrite(PIN_Motor_PWMA, leftSpeed);
  analogWrite(PIN_Motor_PWMB, rightSpeed);
}

void motorStop() {
  analogWrite(PIN_Motor_PWMA, 0);
  analogWrite(PIN_Motor_PWMB, 0);
  currentMode = MODE_STOP;
}
