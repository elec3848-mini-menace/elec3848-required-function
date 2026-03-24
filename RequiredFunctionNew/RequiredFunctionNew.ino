#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// =========================
// OLED
// =========================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// =========================
// CONFIGS
// =========================
#define MAIN_SPEED 60
#define TURN_SPEED 110
#define MOTOR_SYNC_DELAY 250

const float TICKS_PER_CM = 18.0;   // MUST tune on your robot
const int WALL_EQUAL_TOLERANCE_CM = 4;

// =========================
// PINS
// =========================
#define FRONT_LEFT_PWM 12
#define FRONT_LEFT_BACKWARD 34
#define FRONT_LEFT_FORWARD 35
#define FRONT_LEFT_EN_A 18

#define FRONT_RIGHT_PWM 8
#define FRONT_RIGHT_BACKWARD 37
#define FRONT_RIGHT_FORWARD 36
#define FRONT_RIGHT_EN_A 19

#define BACK_LEFT_PWM 9
#define BACK_LEFT_BACKWARD 43
#define BACK_LEFT_FORWARD 42
#define BACK_LEFT_EN_A 3

#define BACK_RIGHT_PWM 5
#define BACK_RIGHT_BACKWARD A4
#define BACK_RIGHT_FORWARD A5
#define BACK_RIGHT_EN_A 2

#define BACK_LEFT_US_TRIG A9
#define BACK_LEFT_US_ECHO A8
#define BACK_RIGHT_US_TRIG A11
#define BACK_RIGHT_US_ECHO A10

// =========================
// MOVEMENT DEFINITIONS
// Fixed so names match switch cases
// =========================
#define moveForward 0
#define moveBackward 1
#define moveLeft 2
#define moveRight 3
#define moveClockwise 4
#define moveCounterclockwise 5

// =========================
// ENCODER / MOTOR SYNC
// =========================
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

unsigned long lastMotorSyncTime = 0;
const unsigned long MOTOR_SYNC_INTERVAL = 1;

bool missionDone = false;

// =========================
// OLED
// =========================
void oledShowText(String text) {
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.print(text);
  display.display();
}

// =========================
// SENSOR HELPERS
// =========================
int getUltrasonicCM(int sensor) {   // 0 = left, 1 = right
  int trigPin = 0;
  int echoPin = 0;

  if (sensor == 0) {
    trigPin = BACK_LEFT_US_TRIG;
    echoPin = BACK_LEFT_US_ECHO;
  } else {
    trigPin = BACK_RIGHT_US_TRIG;
    echoPin = BACK_RIGHT_US_ECHO;
  }

  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 50000);
  int cm = (duration / 2) / 29.1;

  if (cm <= 0) cm = 999;   // avoid bad reading causing wrong turn
  return cm;
}

// =========================
// SETUP FUNCTIONS
// =========================
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

void setupSensors() {
  pinMode(BACK_LEFT_US_TRIG, OUTPUT);
  pinMode(BACK_LEFT_US_ECHO, INPUT);

  pinMode(BACK_RIGHT_US_TRIG, OUTPUT);
  pinMode(BACK_RIGHT_US_ECHO, INPUT);
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

// =========================
// MOTOR CONTROL
// =========================
void moveRobot(int dir, int pwm) {
  switch (dir) {
    case moveForward:
      digitalWrite(FRONT_LEFT_BACKWARD, LOW);
      digitalWrite(FRONT_LEFT_FORWARD, HIGH);

      digitalWrite(FRONT_RIGHT_BACKWARD, HIGH);
      digitalWrite(FRONT_RIGHT_FORWARD, LOW);

      digitalWrite(BACK_LEFT_BACKWARD, LOW);
      digitalWrite(BACK_LEFT_FORWARD, HIGH);

      digitalWrite(BACK_RIGHT_BACKWARD, HIGH);
      digitalWrite(BACK_RIGHT_FORWARD, LOW);
      break;

    case moveBackward:
      digitalWrite(FRONT_LEFT_BACKWARD, HIGH);
      digitalWrite(FRONT_LEFT_FORWARD, LOW);

      digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
      digitalWrite(FRONT_RIGHT_FORWARD, HIGH);

      digitalWrite(BACK_LEFT_BACKWARD, HIGH);
      digitalWrite(BACK_LEFT_FORWARD, LOW);

      digitalWrite(BACK_RIGHT_BACKWARD, LOW);
      digitalWrite(BACK_RIGHT_FORWARD, HIGH);
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

      digitalWrite(FRONT_RIGHT_BACKWARD, LOW);
      digitalWrite(FRONT_RIGHT_FORWARD, HIGH);

      digitalWrite(BACK_LEFT_BACKWARD, HIGH);
      digitalWrite(BACK_LEFT_FORWARD, LOW);

      digitalWrite(BACK_RIGHT_BACKWARD, HIGH);
      digitalWrite(BACK_RIGHT_FORWARD, LOW);
      break;

    case moveCounterclockwise:
      digitalWrite(FRONT_LEFT_BACKWARD, HIGH);
      digitalWrite(FRONT_LEFT_FORWARD, LOW);

      digitalWrite(FRONT_RIGHT_BACKWARD, HIGH);
      digitalWrite(FRONT_RIGHT_FORWARD, LOW);

      digitalWrite(BACK_LEFT_BACKWARD, LOW);
      digitalWrite(BACK_LEFT_FORWARD, HIGH);

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

void motorSyncService() {
  if (millis() - startMoveMillis > MOTOR_SYNC_DELAY) {
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

// =========================
// MOVEMENT HELPERS
// =========================
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

void moveForwardDistanceCM(float distanceCM) {
  long targetTicks = (long)(distanceCM * TICKS_PER_CM);

  resetEncoderCounts();
  moveRobot(moveForward, MAIN_SPEED);

  while (getAverageEncoderCount() < targetTicks) {
    motorSyncService();
    delay(1);
  }

  stopRobot();
}

void alignWithWallUsingUltrasonic() {
  Serial.println("Start wall alignment");

  while (true) {
    int leftDist = getUltrasonicCM(0);
    delay(50); // Give the sound waves a moment to clear
    int rightDist = getUltrasonicCM(1);

    // Filter bad readings
    if (leftDist >= 300 || rightDist >= 300 || leftDist <= 0) {
      stopRobot();
      delay(50);
      continue;
    }

    // Your Exit Condition
    if (abs(leftDist - rightDist) <= WALL_EQUAL_TOLERANCE_CM) {
      stopRobot();
      Serial.println("Wall aligned");
      break;
    }

    if (leftDist > rightDist) {
      // Left side further away -> Turn RIGHT
      Serial.println("Turn RIGHT");
      moveRobot(moveClockwise, TURN_SPEED);
      delay(120); // Your original successful delay
      stopRobot();
    }
    else if (rightDist > leftDist) {
      // Right side further away -> Turn LEFT
      Serial.println("Turn LEFT");
      // FIXED: Actually use Counterclockwise here!
      moveRobot(moveCounterclockwise, TURN_SPEED); 
      delay(120); // Match the RIGHT delay so it doesn't "over-tweak"
      stopRobot();
    }

    delay(150); // Settle time - crucial for clean sensor reads
  }
}

// =========================
// SETUP
// =========================
void setup() {
  Serial.begin(115200);

  setupMotor();
  setupSensors();

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("SSD1306 allocation failed");
  }

  oledShowText("Ready");
  stopRobot();
  delay(1000);
}

// =========================
// LOOP
// =========================
void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - lastMotorSyncTime >= MOTOR_SYNC_INTERVAL) {
    motorSyncService();
    lastMotorSyncTime = currentMillis;
  }

  if (!missionDone) {
    oledShowText("Move 30");
    moveForwardDistanceCM(30.0);
    delay(300);

    oledShowText("Align");
    alignWithWallUsingUltrasonic();
    delay(300);

    oledShowText("Forward");
    moveForwardDistanceCM(30.0);
    delay(300);

    stopRobot();
    oledShowText("Done");
    missionDone = true;
  }
}