/*
  Optional ESP32 + Blynk drive controller for VROD.

  Automatic sequence:
    GPIO4 rising edge  -> scan the camera/ultrasonic servo from 90 to 180 to 0
    GPIO12 rising edge -> freeze the detected angle and align using the MPU6050
    aligned chassis    -> drive forward until <= 15 cm, or until the safety timeout

  Blynk manual controls (available only while the automatic controller is idle):
    V0 forward, V1 backward, V2 left, V3 right

  GPIO12 is a strapping pin on many classic ESP32 boards. Use another suitable input
  and update INPUT_DETECTED_PIN if your board has boot problems. The HC-SR04 ECHO
  output must be reduced to 3.3 V before it reaches GPIO13.
*/

#include "secrets.h"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <ESP32Servo.h>
#include <esp_arduino_version.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

const int INPUT_START_PIN = 4;
const int INPUT_DETECTED_PIN = 12;

const int SERVO_PIN = 18;
const int TRIG_PIN = 5;
const int ECHO_PIN = 13;

const int LEFT_MOTOR_IN1 = 27;
const int LEFT_MOTOR_IN2 = 26;
const int LEFT_MOTOR_ENA = 25;
const int RIGHT_MOTOR_IN3 = 33;
const int RIGHT_MOTOR_IN4 = 32;
const int RIGHT_MOTOR_ENB = 17;

const int I2C_SDA = 21;
const int I2C_SCL = 22;

const int PWM_FREQ = 20000;
const int PWM_RES = 8;
const int PWM_CH_L = 0;
const int PWM_CH_R = 1;

const int SERVO_CENTER = 90;
const unsigned long SERVO_STEP_DELAY = 200UL;
const int MPU_ALIGNMENT_TOLERANCE = 5;
const int MOTOR_SPEED = 150;
const float STOP_DISTANCE_CM = 15.0;
const unsigned long MAX_ALIGNMENT_TIME = 10000UL;
const unsigned long MAX_FORWARD_TIME = 15000UL;

Servo scanServo;
MPU6050 mpu;

int servoPos = SERVO_CENTER;
int servoDirection = 1;
bool servoMoving = false;
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
float targetServoRelativeAngle = 0.0;

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

float getDistanceCM();
void updateServoPosition();
bool initializeMPU();
bool updateMPUReading();
void setupMotorPwm();
void writeLeftPwm(int pwm);
void writeRightPwm(int pwm);
void moveForward();
void turnLeft(int speed);
void turnRight(int speed);
void stopMotors();
void setLeftMotor(int direction, int pwm);
void setRightMotor(int direction, int pwm);

BLYNK_WRITE(V0) {
  if (currentState == IDLE && param.asInt()) moveForward();
  else stopMotors();
}

BLYNK_WRITE(V1) {
  if (currentState == IDLE && param.asInt()) {
    setLeftMotor(-1, MOTOR_SPEED);
    setRightMotor(-1, MOTOR_SPEED);
  } else stopMotors();
}

BLYNK_WRITE(V2) {
  if (currentState == IDLE && param.asInt()) {
    setLeftMotor(-1, MOTOR_SPEED);
    setRightMotor(1, MOTOR_SPEED);
  } else stopMotors();
}

BLYNK_WRITE(V3) {
  if (currentState == IDLE && param.asInt()) {
    setLeftMotor(1, MOTOR_SPEED);
    setRightMotor(-1, MOTOR_SPEED);
  } else stopMotors();
}

void setup() {
  Serial.begin(115200);

  pinMode(INPUT_START_PIN, INPUT);
  pinMode(INPUT_DETECTED_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(LEFT_MOTOR_IN1, OUTPUT);
  pinMode(LEFT_MOTOR_IN2, OUTPUT);
  pinMode(RIGHT_MOTOR_IN3, OUTPUT);
  pinMode(RIGHT_MOTOR_IN4, OUTPUT);

  setupMotorPwm();
  stopMotors();

  scanServo.setPeriodHertz(50);
  scanServo.attach(SERVO_PIN, 500, 2400);
  scanServo.write(servoPos);

  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
  Serial.println("VROD ESP32 drive controller ready.");
}

void loop() {
  Blynk.run();

  bool startSignal = digitalRead(INPUT_START_PIN) == HIGH;
  bool detectedSignal = digitalRead(INPUT_DETECTED_PIN) == HIGH;
  bool startRising = startSignal && !lastStartSignal;
  bool detectedRising = detectedSignal && !lastDetectedSignal;
  lastStartSignal = startSignal;
  lastDetectedSignal = detectedSignal;

  switch (currentState) {
    case IDLE:
      if (startRising && !detectedSignal) {
        stopMotors();
        servoPos = SERVO_CENTER;
        servoDirection = 1;
        scanReachedRightLimit = false;
        servoMoving = true;
        scanServo.write(servoPos);
        lastServoMove = millis();
        currentState = SERVO_SCANNING;
        Serial.println("Scanning started.");
      }
      break;

    case SERVO_SCANNING:
      if (detectedRising) {
        servoMoving = false;
        targetServoRelativeAngle = (float)(servoPos - SERVO_CENTER);
        mpuAngleSet = false;
        if (initializeMPU()) {
          alignmentStartTime = millis();
          currentState = ALIGNING_WITH_MPU;
          Serial.printf("Detection at servo angle %d; aligning.\n", servoPos);
        } else {
          Serial.println("MPU unavailable; movement aborted.");
          currentState = STOPPING;
        }
      } else {
        updateServoPosition();
        if (scanReachedRightLimit && servoPos <= 0) {
          Serial.println("Scan completed without a detection.");
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

        float difference = targetServoRelativeAngle - relativeYaw;
        while (difference > 180.0) difference -= 360.0;
        while (difference < -180.0) difference += 360.0;

        if (fabs(difference) <= MPU_ALIGNMENT_TOLERANCE) {
          stopMotors();
          forwardStartTime = millis();
          currentState = MOVING_FORWARD;
          Serial.println("Alignment complete; moving forward.");
        } else {
          int turnSpeed = fabs(difference) < 15.0 ? MOTOR_SPEED / 2 : MOTOR_SPEED;
          if (difference > 0) turnRight(turnSpeed);
          else turnLeft(turnSpeed);
        }
      }

      if (millis() - alignmentStartTime >= MAX_ALIGNMENT_TIME) {
        Serial.println("Alignment timeout; movement aborted.");
        currentState = STOPPING;
      }
      break;

    case MOVING_FORWARD: {
      moveForward();
      float distance = getDistanceCM();
      if (distance > 0 && distance <= STOP_DISTANCE_CM) {
        Serial.println("Stop distance reached.");
        currentState = STOPPING;
      } else if (millis() - forwardStartTime >= MAX_FORWARD_TIME) {
        Serial.println("Forward-motion timeout reached.");
        currentState = STOPPING;
      }
      break;
    }

    case STOPPING:
      stopMotors();
      servoMoving = false;
      mpuAngleSet = false;
      currentState = IDLE;
      break;
  }

  delay(10);
}

void updateServoPosition() {
  if (!servoMoving || millis() - lastServoMove < SERVO_STEP_DELAY) return;

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

float getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duration == 0) return -1.0;
  return duration * 0.0343 / 2.0;
}

bool initializeMPU() {
  if (mpuInitialized && dmpReady) return true;

  mpu.initialize();
  if (!mpu.testConnection()) return false;

  int status = mpu.dmpInitialize();
  if (status != 0) {
    Serial.printf("MPU DMP initialization failed: %d\n", status);
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

void setupMotorPwm() {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcAttachChannel(LEFT_MOTOR_ENA, PWM_FREQ, PWM_RES, PWM_CH_L);
  ledcAttachChannel(RIGHT_MOTOR_ENB, PWM_FREQ, PWM_RES, PWM_CH_R);
#else
  ledcSetup(PWM_CH_L, PWM_FREQ, PWM_RES);
  ledcSetup(PWM_CH_R, PWM_FREQ, PWM_RES);
  ledcAttachPin(LEFT_MOTOR_ENA, PWM_CH_L);
  ledcAttachPin(RIGHT_MOTOR_ENB, PWM_CH_R);
#endif
  writeLeftPwm(0);
  writeRightPwm(0);
}

void writeLeftPwm(int pwm) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(LEFT_MOTOR_ENA, constrain(pwm, 0, 255));
#else
  ledcWrite(PWM_CH_L, constrain(pwm, 0, 255));
#endif
}

void writeRightPwm(int pwm) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(RIGHT_MOTOR_ENB, constrain(pwm, 0, 255));
#else
  ledcWrite(PWM_CH_R, constrain(pwm, 0, 255));
#endif
}

void setLeftMotor(int direction, int pwm) {
  digitalWrite(LEFT_MOTOR_IN1, direction > 0 ? HIGH : LOW);
  digitalWrite(LEFT_MOTOR_IN2, direction < 0 ? HIGH : LOW);
  writeLeftPwm(direction == 0 ? 0 : pwm);
}

void setRightMotor(int direction, int pwm) {
  digitalWrite(RIGHT_MOTOR_IN3, direction > 0 ? HIGH : LOW);
  digitalWrite(RIGHT_MOTOR_IN4, direction < 0 ? HIGH : LOW);
  writeRightPwm(direction == 0 ? 0 : pwm);
}

void moveForward() {
  setLeftMotor(1, MOTOR_SPEED);
  setRightMotor(1, MOTOR_SPEED);
}

void turnLeft(int speed) {
  setLeftMotor(-1, speed);
  setRightMotor(1, speed);
}

void turnRight(int speed) {
  setLeftMotor(1, speed);
  setRightMotor(-1, speed);
}

void stopMotors() {
  setLeftMotor(0, 0);
  setRightMotor(0, 0);
}
