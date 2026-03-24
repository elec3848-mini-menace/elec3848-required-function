#include "motor.h"
#include "config_pins.h"
#include <Arduino.h>
#include <SPI.h>

volatile long lastMicrosFrontLeft = 0;
volatile long lastMicrosFrontRight = 0;
volatile long lastMicrosBackLeft = 0;
volatile long lastMicrosBackRight = 0;

volatile long currentMicrosFrontLeft = 0;
volatile long currentMicrosFrontRight = 0;
volatile long currentMicrosBackLeft = 0;
volatile long currentMicrosBackRight = 0;

volatile int frontLeftPWM = 0;
volatile int frontRightPWM = 0;
volatile int backLeftPWM = 0;
volatile int backRightPWM = 0;

volatile long startMoveMillis = 0;

volatile long frontLeftCount = 0;
volatile long frontRightCount = 0;
volatile long backLeftCount = 0;
volatile long backRightCount = 0;

static unsigned long lastMotorSyncTime = 0;
static const unsigned long MOTOR_SYNC_INTERVAL = 1;

static void frontLeftEncTrig();
static void frontRightEncTrig();
static void backLeftEncTrig();
static void backRightEncTrig();

void frontLeftEncTrig() {
  currentMicrosFrontLeft = micros() - lastMicrosFrontLeft;
  lastMicrosFrontLeft = micros();
  frontLeftCount++;
}
void frontRightEncTrig() {
  currentMicrosFrontRight = micros() - lastMicrosFrontRight;
  lastMicrosFrontRight = micros();
  frontRightCount++;
}
void backLeftEncTrig() {
  currentMicrosBackLeft = micros() - lastMicrosBackLeft;
  lastMicrosBackLeft = micros();
  backLeftCount++;
}
void backRightEncTrig() {
  currentMicrosBackRight = micros() - lastMicrosBackRight;
  lastMicrosBackRight = micros();
  backRightCount++;
}

void setupMotor() {
  pinMode(FRONT_LEFT_PWM, OUTPUT);
  pinMode(FRONT_LEFT_BACKWARD, OUTPUT);
  pinMode(FRONT_LEFT_FORWARD, OUTPUT);
  pinMode(FRONT_LEFT_EN_A, INPUT);

  pinMode(FRONT_RIGHT_PWM, OUTPUT);
  pinMode(FRONT_RIGHT_BACKWARD, OUTPUT);
  pinMode(FRONT_RIGHT_FORWARD, OUTPUT);
  pinMode(FRONT_RIGHT_EN_A, INPUT);

  pinMode(BACK_LEFT_PWM, OUTPUT);
  pinMode(BACK_LEFT_BACKWARD, OUTPUT);
  pinMode(BACK_LEFT_FORWARD, OUTPUT);
  pinMode(BACK_LEFT_EN_A, INPUT);

  pinMode(BACK_RIGHT_PWM, OUTPUT);
  pinMode(BACK_RIGHT_BACKWARD, OUTPUT);
  pinMode(BACK_RIGHT_FORWARD, OUTPUT);
  pinMode(BACK_RIGHT_EN_A, INPUT);

  attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_EN_A), frontLeftEncTrig, FALLING);
  attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_EN_A), frontRightEncTrig, FALLING);
  attachInterrupt(digitalPinToInterrupt(BACK_LEFT_EN_A), backLeftEncTrig, FALLING);
  attachInterrupt(digitalPinToInterrupt(BACK_RIGHT_EN_A), backRightEncTrig, FALLING);
}

void moveRobot(int dir, int pwm) {
  switch (dir) {
    case moveForward:
      digitalWrite(FRONT_LEFT_BACKWARD, LOW);
      digitalWrite(FRONT_LEFT_FORWARD, HIGH);

      digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
      digitalWrite(FRONT_RIGHT_FORWARD, HIGH);

      digitalWrite(BACK_LEFT_BACKWARD, LOW);
      digitalWrite(BACK_LEFT_FORWARD, HIGH);

      digitalWrite(BACK_RIGHT_BACKWARD, LOW);
      digitalWrite(BACK_RIGHT_FORWARD, HIGH);
      break;

    case moveBackward:
      digitalWrite(FRONT_LEFT_BACKWARD, HIGH);
      digitalWrite(FRONT_LEFT_FORWARD, LOW);

      digitalWrite(FRONT_RIGHT_BACKWARD, HIGH);
      digitalWrite(FRONT_RIGHT_FORWARD, LOW);

      digitalWrite(BACK_LEFT_BACKWARD, HIGH);
      digitalWrite(BACK_LEFT_FORWARD, LOW);

      digitalWrite(BACK_RIGHT_BACKWARD, HIGH);
      digitalWrite(BACK_RIGHT_FORWARD, LOW);
      break;

    case moveLeft:
      digitalWrite(FRONT_LEFT_BACKWARD, HIGH);
      digitalWrite(FRONT_LEFT_FORWARD, LOW);

      digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
      digitalWrite(FRONT_RIGHT_FORWARD, HIGH);

      digitalWrite(BACK_LEFT_BACKWARD, LOW);
      digitalWrite(BACK_LEFT_FORWARD, HIGH);

      digitalWrite(BACK_RIGHT_BACKWARD, HIGH);
      digitalWrite(BACK_RIGHT_FORWARD, LOW);
      break;

    case moveRight:
      digitalWrite(FRONT_LEFT_BACKWARD, LOW);
      digitalWrite(FRONT_LEFT_FORWARD, HIGH);

      digitalWrite(FRONT_RIGHT_BACKWARD, HIGH);
      digitalWrite(FRONT_RIGHT_FORWARD, LOW);

      digitalWrite(BACK_LEFT_BACKWARD, HIGH);
      digitalWrite(BACK_LEFT_FORWARD, LOW);

      digitalWrite(BACK_RIGHT_BACKWARD, LOW);
      digitalWrite(BACK_RIGHT_FORWARD, HIGH);
      break;

    case moveClockwise:
      digitalWrite(FRONT_LEFT_BACKWARD, LOW);
      digitalWrite(FRONT_LEFT_FORWARD, HIGH);

      digitalWrite(FRONT_RIGHT_BACKWARD, HIGH);
      digitalWrite(FRONT_RIGHT_FORWARD, LOW);

      digitalWrite(BACK_LEFT_BACKWARD, LOW);
      digitalWrite(BACK_LEFT_FORWARD, HIGH);

      digitalWrite(BACK_RIGHT_BACKWARD, HIGH);
      digitalWrite(BACK_RIGHT_FORWARD, LOW);
      break;

    case moveCounterclockwise:
      digitalWrite(FRONT_LEFT_BACKWARD, HIGH);
      digitalWrite(FRONT_LEFT_FORWARD, LOW);

      digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
      digitalWrite(FRONT_RIGHT_FORWARD, HIGH);

      digitalWrite(BACK_LEFT_BACKWARD, HIGH);
      digitalWrite(BACK_LEFT_FORWARD, LOW);

      digitalWrite(BACK_RIGHT_BACKWARD, LOW);
      digitalWrite(BACK_RIGHT_FORWARD, HIGH);
      break;
  }

  frontLeftPWM = pwm;
  frontRightPWM = pwm;
  backLeftPWM = pwm;
  backRightPWM = pwm;

  analogWrite(FRONT_LEFT_PWM, frontLeftPWM);
  analogWrite(FRONT_RIGHT_PWM, frontRightPWM);
  analogWrite(BACK_LEFT_PWM, backLeftPWM);
  analogWrite(BACK_RIGHT_PWM, backRightPWM);

  startMoveMillis = millis();
}

void stopRobot() {
  digitalWrite(FRONT_LEFT_BACKWARD, LOW);
  digitalWrite(FRONT_LEFT_FORWARD, LOW);
  digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
  digitalWrite(FRONT_RIGHT_FORWARD, LOW);
  digitalWrite(BACK_LEFT_BACKWARD, LOW);
  digitalWrite(BACK_LEFT_FORWARD, LOW);
  digitalWrite(BACK_RIGHT_BACKWARD, LOW);
  digitalWrite(BACK_RIGHT_FORWARD, LOW);

  analogWrite(FRONT_LEFT_PWM, 0);
  analogWrite(FRONT_RIGHT_PWM, 0);
  analogWrite(BACK_LEFT_PWM, 0);
  analogWrite(BACK_RIGHT_PWM, 0);
}

void resetEncoderCounts() {
  noInterrupts();
  frontLeftCount = 0;
  frontRightCount = 0;
  backLeftCount = 0;
  backRightCount = 0;
  interrupts();
}

long getAverageEncoderCount() {
  noInterrupts();
  long total = frontLeftCount + frontRightCount + backLeftCount + backRightCount;
  interrupts();
  return total / 4;
}

// --- New functions for individual encoder counts ---
long getFrontLeftEncoderCount() {
  noInterrupts();
  long count = frontLeftCount;
  interrupts();
  return count;
}

long getFrontRightEncoderCount() {
  noInterrupts();
  long count = frontRightCount;
  interrupts();
  return count;
}

long getBackLeftEncoderCount() {
  noInterrupts();
  long count = backLeftCount;
  interrupts();
  return count;
}

long getBackRightEncoderCount() {
  noInterrupts();
  long count = backRightCount;
  interrupts();
  return count;
}
// --- End new functions ---

void motorSyncService() {
  if (millis() - startMoveMillis > MOTOR_SYNC_DELAY) {

    // Note: This motor synchronization logic uses frontLeft as a reference.
    // It adjusts PWMs to try and keep speeds consistent.
    // This is separate from localization, which uses the cumulative counts.

    if (currentMicrosFrontRight > currentMicrosFrontLeft) {
      frontRightPWM++;
    } else if (currentMicrosFrontRight < currentMicrosFrontLeft) {
      frontRightPWM--;
    }

    if (currentMicrosBackLeft > currentMicrosFrontLeft) {
      backLeftPWM++;
    } else if (currentMicrosBackLeft < currentMicrosFrontLeft) {
      backLeftPWM--;
    }

    if (currentMicrosBackRight > currentMicrosFrontLeft) {
      backRightPWM++;
    } else if (currentMicrosBackRight < currentMicrosFrontLeft) {
      backRightPWM--;
    }

    frontRightPWM = constrain(frontRightPWM, 0, 255);
    backLeftPWM = constrain(backLeftPWM, 0, 255);
    backRightPWM = constrain(backRightPWM, 0, 255);

    analogWrite(FRONT_RIGHT_PWM, frontRightPWM);
    analogWrite(BACK_LEFT_PWM, backLeftPWM);
    analogWrite(BACK_RIGHT_PWM, backRightPWM);
  }
}