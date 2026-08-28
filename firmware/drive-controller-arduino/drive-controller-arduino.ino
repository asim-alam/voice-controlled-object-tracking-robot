/*
  Primary Arduino drive controller for VROD.

  D4 rising edge  -> scan the camera/ultrasonic servo from 90 to 180 to 0
  D12 rising edge -> freeze the detected angle and align with the MPU6050
  aligned chassis -> drive forward until <= 15 cm or the safety timeout
*/

#include <Servo.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

#define INPUT_START_PIN 4
#define INPUT_DETECTED_PIN 12
#define SERVO_PIN A0
#define TRIG_PIN 5
#define ECHO_PIN 3

#define LEFT_MOTOR_IN1 6
#define LEFT_MOTOR_IN2 7
#define LEFT_MOTOR_ENA 9
#define RIGHT_MOTOR_IN3 8
#define RIGHT_MOTOR_IN4 10
#define RIGHT_MOTOR_ENB 11

const int SERVO_CENTER = 90;
const unsigned long SERVO_STEP_DELAY = 200UL;
const float ALIGNMENT_TOLERANCE = 10.0;
const int MOTOR_SPEED = 150;
const float STOP_DISTANCE_CM = 15.0;
const unsigned long MAX_ALIGNMENT_TIME = 10000UL;
const unsigned long MAX_FORWARD_TIME = 15000UL;

Servo scanServo;
MPU6050 mpu;

int servoPos = SERVO_CENTER;
int servoDirection = 1;
bool servoScanning = false;
bool scanReachedRightLimit = false;
unsigned long lastServoMove = 0;

bool dmpReady = false;
bool mpuInitialized = false;
uint8_t fifoBuffer[64];
Quaternion q;
VectorFloat gravity;
float ypr[3];
bool mpuAngleSet = false;
float initialMpuAngle = 0.0;
float targetAngle = 0.0;

enum SystemState {
  IDLE,
  SERVO_SCANNING,
  ALIGNING_WITH_MPU,
  MOVING_FORWARD,
  STOPPING
};

SystemState currentState = IDLE;
bool lastStartSignal = false;
bool lastDetectedSignal = false;
unsigned long alignmentStartTime = 0;
unsigned long forwardStartTime = 0;

void startServoScan();
void updateServoPosition();
bool initializeMPU();
bool updateMPUReading();
float getDistance();
void moveForward();
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  pinMode(INPUT_START_PIN, INPUT);
  pinMode(INPUT_DETECTED_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(LEFT_MOTOR_ENA, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);
  pinMode(RIGHT_MOTOR_ENB, OUTPUT);

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_CENTER);
  stopMotors();

  Serial.println(F("VROD Arduino drive controller ready"));
}

void loop() {
  bool startSignal = digitalRead(INPUT_START_PIN) == HIGH;
  bool detectedSignal = digitalRead(INPUT_DETECTED_PIN) == HIGH;
  bool startRising = startSignal && !lastStartSignal;
  bool detectedRising = detectedSignal && !lastDetectedSignal;
  lastStartSignal = startSignal;
  lastDetectedSignal = detectedSignal;

  switch (currentState) {
    case IDLE:
      if (startRising && !detectedSignal) {
        startServoScan();
        currentState = SERVO_SCANNING;
      }
      break;

    case SERVO_SCANNING:
      if (detectedRising) {
        servoScanning = false;
        targetAngle = (float)(servoPos - SERVO_CENTER);
        mpuAngleSet = false;
        Serial.print(F("Detection at relative angle: "));
        Serial.println(targetAngle, 1);

        if (initializeMPU()) {
          alignmentStartTime = millis();
          currentState = ALIGNING_WITH_MPU;
        } else {
          Serial.println(F("MPU unavailable; movement aborted"));
          currentState = STOPPING;
        }
      } else {
        updateServoPosition();
        if (scanReachedRightLimit && servoPos <= 0) {
          Serial.println(F("Scan completed without a detection"));
          currentState = STOPPING;
        }
      }
      break;

    case ALIGNING_WITH_MPU:
      if (updateMPUReading()) {
        float currentYaw = ypr[0] * 180.0 / M_PI;
        if (!mpuAngleSet) {
          initialMpuAngle = currentYaw;
          mpuAngleSet = true;
        }

        float relativeYaw = currentYaw - initialMpuAngle;
        while (relativeYaw > 180.0) relativeYaw -= 360.0;
        while (relativeYaw < -180.0) relativeYaw += 360.0;

        float difference = targetAngle - relativeYaw;
        while (difference > 180.0) difference -= 360.0;
        while (difference < -180.0) difference += 360.0;

        if (abs(difference) <= ALIGNMENT_TOLERANCE) {
          stopMotors();
          forwardStartTime = millis();
          currentState = MOVING_FORWARD;
          Serial.println(F("Alignment complete; moving forward"));
        } else if (difference > 0) {
          turnRight(MOTOR_SPEED);
        } else {
          turnLeft(MOTOR_SPEED);
        }
      }

      if (millis() - alignmentStartTime >= MAX_ALIGNMENT_TIME) {
        Serial.println(F("Alignment timeout; movement aborted"));
        currentState = STOPPING;
      }
      break;

    case MOVING_FORWARD: {
      moveForward();
      float distance = getDistance();
      if (distance > 0 && distance <= STOP_DISTANCE_CM) {
        Serial.println(F("Stop distance reached"));
        currentState = STOPPING;
      } else if (millis() - forwardStartTime >= MAX_FORWARD_TIME) {
        Serial.println(F("Forward-motion timeout reached"));
        currentState = STOPPING;
      }
      break;
    }

    case STOPPING:
      stopMotors();
      servoScanning = false;
      mpuAngleSet = false;
      currentState = IDLE;
      break;
  }
}

void startServoScan() {
  stopMotors();
  servoPos = SERVO_CENTER;
  servoDirection = 1;
  scanReachedRightLimit = false;
  servoScanning = true;
  scanServo.write(servoPos);
  lastServoMove = millis();
  Serial.println(F("Scanning started"));
}

void updateServoPosition() {
  if (!servoScanning || millis() - lastServoMove < SERVO_STEP_DELAY) return;

  servoPos += servoDirection;
  if (servoPos >= 180) {
    servoPos = 180;
    servoDirection = -1;
    scanReachedRightLimit = true;
  } else if (servoPos <= 0) {
    servoPos = 0;
  }

  scanServo.write(servoPos);
  lastServoMove = millis();
}

bool initializeMPU() {
  if (mpuInitialized && dmpReady) return true;

  mpu.initialize();
  if (!mpu.testConnection()) return false;

  uint8_t status = mpu.dmpInitialize();
  if (status != 0) {
    Serial.print(F("MPU DMP initialization failed: "));
    Serial.println(status);
    return false;
  }

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
  dmpReady = true;
  mpuInitialized = true;
  return true;
}

bool updateMPUReading() {
  if (!dmpReady || !mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) return false;
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  return true;
}

float getDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0;
  return duration * 0.0343 / 2.0;
}

void moveForward() {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, MOTOR_SPEED);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, MOTOR_SPEED);
}

void turnLeft(int speed) {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, HIGH);
  analogWrite(LEFT_MOTOR_ENA, speed);
  digitalWrite(RIGHT_MOTOR_IN3, HIGH);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, speed);
}

void turnRight(int speed) {
  digitalWrite(LEFT_MOTOR_IN1, HIGH);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, speed);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, HIGH);
  analogWrite(RIGHT_MOTOR_ENB, speed);
}

void stopMotors() {
  digitalWrite(LEFT_MOTOR_IN1, LOW);
  digitalWrite(LEFT_MOTOR_IN2, LOW);
  analogWrite(LEFT_MOTOR_ENA, 0);
  digitalWrite(RIGHT_MOTOR_IN3, LOW);
  digitalWrite(RIGHT_MOTOR_IN4, LOW);
  analogWrite(RIGHT_MOTOR_ENB, 0);
}
