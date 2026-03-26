#include "motor.h"
#include "config_pins.h"
#include <Arduino.h>
#include <SPI.h>

// Volatile variables for encoder counts and speed measurement
volatile long frontLeftCount = 0;
volatile long frontRightCount = 0;
volatile long backLeftCount = 0;
volatile long backRightCount = 0;

volatile long lastMicrosFrontLeft = 0;
volatile long lastMicrosFrontRight = 0;
volatile long lastMicrosBackLeft = 0;
volatile long lastMicrosBackRight = 0;

volatile long currentMicrosFrontLeft = 0;
volatile long currentMicrosFrontRight = 0;
volatile long currentMicrosBackLeft = 0;
volatile long currentMicrosBackRight = 0;

// Volatile variables for motor PWM values (adjusted by sync service)
volatile int frontLeftPWM = 0;
volatile int frontRightPWM = 0;
volatile int backLeftPWM = 0;
volatile int backRightPWM = 0;

volatile long startMoveMillis = 0; // Time when movement started, used for MOTOR_SYNC_DELAY

// Variables to store previous state of EN_A for quadrature decoding
volatile int lastFrontLeftStateA = LOW;
volatile int lastFrontRightStateA = LOW;
volatile int lastBackLeftStateA = LOW;
volatile int lastBackRightStateA = LOW;

// ISRs for each motor's encoder
void frontLeftEncTrig() {
    currentMicrosFrontLeft = micros() - lastMicrosFrontLeft;
    lastMicrosFrontLeft = micros();

    int currentStateA = digitalRead(FRONT_LEFT_EN_A);
    int currentStateB = digitalRead(FRONT_LEFT_EN_B);

    // Quadrature decoding logic
    if (currentStateA != lastFrontLeftStateA) {
        if (currentStateA == HIGH) {
            if (currentStateB == LOW) {
                frontLeftCount++;
            } else {
                frontLeftCount--;
            }
        } else {
            if (currentStateB == HIGH) { 
                frontLeftCount++;
            } else {
                frontLeftCount--;
            }
        }
    }
    lastFrontLeftStateA = currentStateA; // Update last state
}

void frontRightEncTrig() {
    currentMicrosFrontRight = micros() - lastMicrosFrontRight;
    lastMicrosFrontRight = micros();

    int currentStateA = digitalRead(FRONT_RIGHT_EN_A);
    int currentStateB = digitalRead(FRONT_RIGHT_EN_B);

    if (currentStateA != lastFrontRightStateA) {
        if (currentStateA == HIGH) {
            if (currentStateB == LOW) {
                frontRightCount++;
            } else {
                frontRightCount--;
            }
        } else {
            if (currentStateB == HIGH) {
                frontRightCount++;
            } else {
                frontRightCount--;
            }
        }
    }
    lastFrontRightStateA = currentStateA;
}

void backLeftEncTrig() {
    currentMicrosBackLeft = micros() - lastMicrosBackLeft;
    lastMicrosBackLeft = micros();

    int currentStateA = digitalRead(BACK_LEFT_EN_A);
    int currentStateB = digitalRead(BACK_LEFT_EN_B);

    if (currentStateA != lastBackLeftStateA) {
        if (currentStateA == HIGH) {
            if (currentStateB == LOW) {
                backLeftCount++;
            } else {
                backLeftCount--;
            }
        } else {
            if (currentStateB == HIGH) {
                backLeftCount++;
            } else {
                backLeftCount--;
            }
        }
    }
    lastBackLeftStateA = currentStateA;
}

void backRightEncTrig() {
    currentMicrosBackRight = micros() - lastMicrosBackRight;
    lastMicrosBackRight = micros();

    int currentStateA = digitalRead(BACK_RIGHT_EN_A);
    int currentStateB = digitalRead(BACK_RIGHT_EN_B);

    if (currentStateA != lastBackRightStateA) {
        if (currentStateA == HIGH) {
            if (currentStateB == LOW) {
                backRightCount++;
            } else {
                backRightCount--;
            }
        } else {
            if (currentStateB == HIGH) {
                backRightCount++;
            } else {
                backRightCount--;
            }
        }
    }
    lastBackRightStateA = currentStateA;
}


void setupMotor() {
    // Setup PWM and Direction pins
    pinMode(FRONT_LEFT_PWM, OUTPUT);
    pinMode(FRONT_LEFT_BACKWARD, OUTPUT);
    pinMode(FRONT_LEFT_FORWARD, OUTPUT);

    pinMode(FRONT_RIGHT_PWM, OUTPUT);
    pinMode(FRONT_RIGHT_BACKWARD, OUTPUT);
    pinMode(FRONT_RIGHT_FORWARD, OUTPUT);

    pinMode(BACK_LEFT_PWM, OUTPUT);
    pinMode(BACK_LEFT_BACKWARD, OUTPUT);
    pinMode(BACK_LEFT_FORWARD, OUTPUT);

    pinMode(BACK_RIGHT_PWM, OUTPUT);
    pinMode(BACK_RIGHT_BACKWARD, OUTPUT);
    pinMode(BACK_RIGHT_FORWARD, OUTPUT);

    // Setup Encoder A and B pins as inputs
    pinMode(FRONT_LEFT_EN_A, INPUT);
    pinMode(FRONT_LEFT_EN_B, INPUT);

    pinMode(FRONT_RIGHT_EN_A, INPUT);
    pinMode(FRONT_RIGHT_EN_B, INPUT);

    pinMode(BACK_LEFT_EN_A, INPUT);
    pinMode(BACK_LEFT_EN_B, INPUT);

    pinMode(BACK_RIGHT_EN_A, INPUT);
    pinMode(BACK_RIGHT_EN_B, INPUT);

    // Attach interrupts for EN_A pins on CHANGE (both rising and falling edges)
    attachInterrupt(digitalPinToInterrupt(FRONT_LEFT_EN_A), frontLeftEncTrig, CHANGE);
    attachInterrupt(digitalPinToInterrupt(FRONT_RIGHT_EN_A), frontRightEncTrig, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BACK_LEFT_EN_A), backLeftEncTrig, CHANGE);
    attachInterrupt(digitalPinToInterrupt(BACK_RIGHT_EN_A), backRightEncTrig, CHANGE);

    Serial.println("Motor setup complete. Encoders initialized.");
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

    frontLeftPWM = 0;
    frontRightPWM = 0;
    backLeftPWM = 0;
    backRightPWM = 0;

    analogWrite(FRONT_LEFT_PWM, frontLeftPWM);
    analogWrite(FRONT_RIGHT_PWM, frontRightPWM);
    analogWrite(BACK_LEFT_PWM, backLeftPWM);
    analogWrite(BACK_RIGHT_PWM, backRightPWM);
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

void motorSyncService() {
    if (millis() - startMoveMillis > MOTOR_SYNC_DELAY) {

        // Note: This motor synchronization logic uses frontLeft as a reference.
        // It adjusts PWMs to try and keep speeds consistent.
        // currentMicros is time since last tick. Smaller micros = faster motor.

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

        // Constrain PWM values to valid range (0-255)
        frontRightPWM = constrain(frontRightPWM, 0, 255);
        backLeftPWM = constrain(backLeftPWM, 0, 255);
        backRightPWM = constrain(backRightPWM, 0, 255);

        // Apply adjusted PWM values
        analogWrite(FRONT_RIGHT_PWM, frontRightPWM);
        analogWrite(BACK_LEFT_PWM, backLeftPWM);
        analogWrite(BACK_RIGHT_PWM, backRightPWM);
    }
}

long getAverageAbsoluteEncoderCount() {
    noInterrupts();
    long fl = frontLeftCount;
    long fr = frontRightCount;
    long bl = backLeftCount;
    long br = backRightCount;
    interrupts();

    fl = abs(fl);
    fr = abs(fr);
    bl = abs(bl);
    br = abs(br);

    return (fl + fr + bl + br) / 4;
}