#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Servo.h>

// ======================================================
// OLED
// ======================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ======================================================
// Original variables
// ======================================================
int oldV = 1, newV = 0;
int pan = 90;
int tilt = 120;
int window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
unsigned long timeNow = 0;

Servo servo_pan;
Servo servo_tilt;

int servo_min = 20;
int servo_max = 160;

// ======================================================
// Mission tuning parameters
// ======================================================
int Motor_PWM = 110;  // analogWrite uses 0~255 on Mega

// const int SPEED_APPROACH = 100;
// const int SPEED_ALIGN    = 90;
// const int SPEED_STRAFE   = 95;
// const int SPEED_ROTATE   = 85;
// const int SPEED_FINAL    = 75;

const int SPEED_APPROACH = 255;
const int SPEED_ALIGN    = 255;
const int SPEED_STRAFE   = 255;
const int SPEED_ROTATE   = 255;
const int SPEED_FINAL    = 255;

// ultrasonic targets
const int TARGET_APPROACH_WALL_CM = 22;   // near pad area before final move
const int TARGET_FINAL_WALL_CM    = 8;    // final parking distance
const int PARALLEL_TOL_CM         = 2;    // left/right ultrasonic difference tolerance
const int DIST_TOL_CM             = 1;

// light tuning
const int LIGHT_DIFF_TOL = 900;            // left/right balance tolerance
const int LIGHT_MIN_SUM  = 120;           // minimum light sum to consider light is seen
const int LIGHT_DROP_THRESHOLD = 35;      // brightness drop for "passed target"

// color tuning
// NOTE: TCS3200 usually gives LOWER pulse count when that color is stronger.
// These thresholds may need tuning on your real pad.
const int COLOR_SAMPLE_COUNT = 10;
const float RED_RATIO_THRESH   = 0.85;    // redFreq < greenFreq*0.85 => red
const float GREEN_RATIO_THRESH = 0.85;    // greenFreq < redFreq*0.85 => green

// timeouts
const unsigned long APPROACH_TIMEOUT = 25000;
const unsigned long ALIGN_TIMEOUT    = 8000;
const unsigned long SEARCH_TIMEOUT   = 18000;

// ======================================================
// Runtime sensor variables
// ======================================================
int lightLeft = 0, lightRight = 0;
long distLeft = 0, distRight = 0;
int redFreq = 0, greenFreq = 0;

bool missionStarted = false;
bool missionDone = false;

// ======================================================
// Motor pins
// ======================================================
#define PWMA 12  //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction

#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction

#define PWMC 9    //Motor C PWM --> from 6 to 9
#define DIRC1 43
#define DIRC2 42  //Motor C Direction

#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26
#define DIRD2 A5  //27  //Motor D Direction

// ======================================================
// Sensor pins
// ======================================================
#define LDR_LEFT A2
#define LDR_RIGHT A3

#define TRIG_LEFT A9
#define ECHO_LEFT A8
#define TRIG_RIGHT A11
#define ECHO_RIGHT A10

// TCS3200 Color Sensor
#define S0 A12
#define S1 A13
#define S2 A14
#define S3 A15
#define sensorOut A7

// ======================================================
// Serial
// ======================================================
#define SERIAL Serial
#define BTSERIAL Serial3

// ======================================================
// Motor macros
// ======================================================
#define MOTORA_FORWARD(pwm) \
  do { digitalWrite(DIRA1, LOW); digitalWrite(DIRA2, HIGH); analogWrite(PWMA, pwm); } while (0)
#define MOTORA_STOP(x) \
  do { digitalWrite(DIRA1, LOW); digitalWrite(DIRA2, LOW); analogWrite(PWMA, 0); } while (0)
#define MOTORA_BACKOFF(pwm) \
  do { digitalWrite(DIRA1, HIGH); digitalWrite(DIRA2, LOW); analogWrite(PWMA, pwm); } while (0)

#define MOTORB_FORWARD(pwm) \
  do { digitalWrite(DIRB1, LOW); digitalWrite(DIRB2, HIGH); analogWrite(PWMB, pwm); } while (0)
#define MOTORB_STOP(x) \
  do { digitalWrite(DIRB1, LOW); digitalWrite(DIRB2, LOW); analogWrite(PWMB, 0); } while (0)
#define MOTORB_BACKOFF(pwm) \
  do { digitalWrite(DIRB1, HIGH); digitalWrite(DIRB2, LOW); analogWrite(PWMB, pwm); } while (0)

#define MOTORC_FORWARD(pwm) \
  do { digitalWrite(DIRC1, LOW); digitalWrite(DIRC2, HIGH); analogWrite(PWMC, pwm); } while (0)
#define MOTORC_STOP(x) \
  do { digitalWrite(DIRC1, LOW); digitalWrite(DIRC2, LOW); analogWrite(PWMC, 0); } while (0)
#define MOTORC_BACKOFF(pwm) \
  do { digitalWrite(DIRC1, HIGH); digitalWrite(DIRC2, LOW); analogWrite(PWMC, pwm); } while (0)

#define MOTORD_FORWARD(pwm) \
  do { digitalWrite(DIRD1, LOW); digitalWrite(DIRD2, HIGH); analogWrite(PWMD, pwm); } while (0)
#define MOTORD_STOP(x) \
  do { digitalWrite(DIRD1, LOW); digitalWrite(DIRD2, LOW); analogWrite(PWMD, 0); } while (0)
#define MOTORD_BACKOFF(pwm) \
  do { digitalWrite(DIRD1, HIGH); digitalWrite(DIRD2, LOW); analogWrite(PWMD, pwm); } while (0)

// ======================================================
// Enums
// ======================================================
enum PadColor {
  PAD_UNKNOWN = 0,
  PAD_RED,
  PAD_GREEN
};

// ======================================================
// Motion functions from your code
// ======================================================
//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK() {
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE() {
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2() {
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2() {
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

// rotate one way
void rotate_1() {
  MOTORA_BACKOFF(Motor_PWM);
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM);
  MOTORD_BACKOFF(Motor_PWM);
}

// rotate the other way
void rotate_2() {
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
}

void STOP() {
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

// ======================================================
// Helper: OLED
// ======================================================
void oledShow(const String &line1, const String &line2 = "") {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(line1);
  if (line2.length() > 0) display.println(line2);
  display.display();
}

// ======================================================
// Helper: sensor reading
// ======================================================
long readUltrasonic(int trig, int echo) {
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);

  long duration = pulseIn(echo, HIGH, 50000);
  if (duration == 0) return 999;  // timeout / invalid
  return (long)((duration * 0.034) / 2.0);
}

void readLightSensors() {
  lightLeft = analogRead(LDR_LEFT);
  lightRight = analogRead(LDR_RIGHT);
}

void readUltrasonicSensors() {
  distLeft = readUltrasonic(TRIG_LEFT, ECHO_LEFT);
  distRight = readUltrasonic(TRIG_RIGHT, ECHO_RIGHT);
}

void readColorSensorsRaw() {
  // RED
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  delay(5);
  redFreq = pulseIn(sensorOut, LOW, 30000);

  // GREEN
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  delay(5);
  greenFreq = pulseIn(sensorOut, LOW, 30000);
}

void readAllSensors() {
  readLightSensors();
  readUltrasonicSensors();
  readColorSensorsRaw();
}

long wallDistanceAvg() {
  readUltrasonicSensors();
  if (distLeft == 999 && distRight == 999) return 999;
  return (distLeft + distRight) / 2;
}

int lightSum() {
  readLightSensors();
  return lightLeft + lightRight;
}

int lightDiff() {
  readLightSensors();
  return lightLeft - lightRight;
}

// ======================================================
// Helper: logging
// ======================================================
void printSensorLog() {
  Serial.print("LDR L:");
  Serial.print(lightLeft);
  Serial.print(" R:");
  Serial.print(lightRight);
  Serial.print(" | US L:");
  Serial.print(distLeft);
  Serial.print("cm R:");
  Serial.print(distRight);
  Serial.print("cm | RED:");
  Serial.print(redFreq);
  Serial.print(" GREEN:");
  Serial.println(greenFreq);
}

// ======================================================
// Color detection
// ======================================================
PadColor detectPadColor() {
  long redSum = 0;
  long greenSum = 0;

  for (int i = 0; i < COLOR_SAMPLE_COUNT; i++) {
    readColorSensorsRaw();
    if (redFreq > 0) redSum += redFreq;
    if (greenFreq > 0) greenSum += greenFreq;
    delay(20);
  }

  float redAvg = redSum / (float)COLOR_SAMPLE_COUNT;
  float greenAvg = greenSum / (float)COLOR_SAMPLE_COUNT;

  Serial.print("Color avg -> RED:");
  Serial.print(redAvg);
  Serial.print(" GREEN:");
  Serial.println(greenAvg);

  // lower frequency/pulse count usually means stronger detected color
  if (redAvg < greenAvg * RED_RATIO_THRESH) {
    Serial.println("Detected PAD_RED");
    return PAD_RED;
  }
  if (greenAvg < redAvg * GREEN_RATIO_THRESH) {
    Serial.println("Detected PAD_GREEN");
    return PAD_GREEN;
  }

  Serial.println("Detected PAD_UNKNOWN");
  return PAD_UNKNOWN;
}

// ======================================================
// Motion wrappers
// ======================================================
void setSpeed(int pwm) {
  Motor_PWM = constrain(pwm, 0, 255);
}

void moveForward(int pwm) {
  setSpeed(pwm);
  ADVANCE();
}

void moveBackward(int pwm) {
  setSpeed(pwm);
  BACK();
}

void moveLeft(int pwm) {
  setSpeed(pwm);
  LEFT_2();
}

void moveRight(int pwm) {
  setSpeed(pwm);
  RIGHT_2();
}

void rotateLeft(int pwm) {
  setSpeed(pwm);
  rotate_1();
}

void rotateRight(int pwm) {
  setSpeed(pwm);
  rotate_2();
}

// ======================================================
// Light tracking helpers
// ======================================================
bool centerOnCurrentLight(unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    readLightSensors();
    int diff = lightLeft - lightRight;
    int sumv = lightLeft + lightRight;

    printSensorLog();

    if (sumv < LIGHT_MIN_SUM) {
      // no strong light, slowly rotate to search
      rotateRight(SPEED_ROTATE);
      delay(80);
      STOP();
      delay(40);
      continue;
    }

    if (abs(diff) <= LIGHT_DIFF_TOL) {
      STOP();
      return true;
    }

    if (diff > 0) {
      // left LDR brighter -> rotate left
      rotateLeft(SPEED_ROTATE);
    } else {
      // right LDR brighter -> rotate right
      rotateRight(SPEED_ROTATE);
    }

    delay(60);
    STOP();
    delay(30);
  }

  STOP();
  return false;
}

// ======================================================
// Keep robot parallel to wall
// ======================================================
bool makeParallelToWall(unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    readUltrasonicSensors();
    printSensorLog();

    if (distLeft == 999 || distRight == 999) {
      STOP();
      delay(50);
      continue;
    }

    long diff = distLeft - distRight;

    if (abs(diff) <= PARALLEL_TOL_CM) {
      STOP();
      return true;
    }

    if (diff > 0) {
      // left farther than right
      rotateRight(SPEED_ALIGN);
    } else {
      // right farther than left
      rotateLeft(SPEED_ALIGN);
    }

    delay(70);
    STOP();
    delay(40);
  }

  STOP();
  return false;
}

// ======================================================
// Move to specific wall distance while staying safe
// ======================================================
bool moveToWallDistance(int targetCm, unsigned long timeoutMs) {
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    readUltrasonicSensors();
    long avgDist = wallDistanceAvg();
    printSensorLog();

    if (avgDist == 999) {
      STOP();
      delay(50);
      continue;
    }

    long err = avgDist - targetCm;

    if (abs(err) <= DIST_TOL_CM) {
      STOP();
      return true;
    }

    // keep parallel first
    if (abs(distLeft - distRight) > PARALLEL_TOL_CM) {
      makeParallelToWall(1200);
      continue;
    }

    if (err > 0) {
      // too far from wall
      moveForward(SPEED_FINAL);
    } else {
      // too near the wall
      moveBackward(SPEED_FINAL);
    }

    delay(80);
    STOP();
    delay(40);
  }

  STOP();
  return false;
}

// ======================================================
// Approach middle light toward color pad area
// ======================================================
bool approachMiddleLightAndPad() {
  oledShow("Mission:", "Middle light");
  Serial.println("=== APPROACH MIDDLE LIGHT ===");

  unsigned long start = millis();

  while (millis() - start < APPROACH_TIMEOUT) {
    centerOnCurrentLight(800);
    readAllSensors();
    printSensorLog();

    long avgDist = wallDistanceAvg();

    // stop condition: near wall/pad area
    if (avgDist <= TARGET_APPROACH_WALL_CM) {
      STOP();
      Serial.println("Reached color pad area.");
      return true;
    }

    // if not centered, re-center first
    int diff = lightLeft - lightRight;
    int sumv = lightLeft + lightRight;

    if (sumv < LIGHT_MIN_SUM) {
      rotateRight(SPEED_ROTATE);
      delay(100);
      STOP();
      delay(40);
      continue;
    }

    if (abs(diff) > LIGHT_DIFF_TOL) {
      continue;
    }

    Serial.println("moving");

    moveForward(SPEED_APPROACH);
    delay(120);
    STOP();
    delay(40);
  }

  STOP();
  Serial.println("Approach timeout.");
  return false;
}

// ======================================================
// Search sideways for side parking light
// Red  -> right
// Green-> left
//
// Strategy:
// keep near wall
// strafe in chosen direction
// watch total brightness
// when brightness peaks then drops, target area is reached
// ======================================================
bool searchSideLightAndStop(bool goRight) {
  Serial.println("=== SEARCH SIDE LIGHT ===");
  oledShow(goRight ? "Pad RED" : "Pad GREEN",
           goRight ? "Go RIGHT" : "Go LEFT");

  unsigned long start = millis();
  int maxBrightness = 0;
  bool sawStrongLight = false;

  while (millis() - start < SEARCH_TIMEOUT) {
    // keep parallel often
    makeParallelToWall(700);

    // keep around the near-wall search distance first
    moveToWallDistance(TARGET_APPROACH_WALL_CM, 1200);

    readLightSensors();
    int total = lightLeft + lightRight;
    printSensorLog();

    if (total > maxBrightness) {
      maxBrightness = total;
    }

    if (total > LIGHT_MIN_SUM) {
      sawStrongLight = true;
    }

    // if already saw bright target and now it dropped, assume passed peak
    if (sawStrongLight && total < (maxBrightness - LIGHT_DROP_THRESHOLD)) {
      STOP();
      Serial.println("Target side light peak passed. Stop search.");
      return true;
    }

    if (goRight) {
      moveRight(SPEED_STRAFE);
    } else {
      moveLeft(SPEED_STRAFE);
    }

    delay(100);
    STOP();
    delay(40);
  }

  STOP();
  Serial.println("Side light search timeout.");
  return false;
}

// ======================================================
// Final parking
// ======================================================
bool finalParkingAt8cm() {
  Serial.println("=== FINAL PARKING ===");
  oledShow("Final align", "8cm");

  bool ok1 = makeParallelToWall(ALIGN_TIMEOUT);
  bool ok2 = moveToWallDistance(TARGET_FINAL_WALL_CM, ALIGN_TIMEOUT);
  bool ok3 = makeParallelToWall(ALIGN_TIMEOUT);

  STOP();

  Serial.print("Final align results: ");
  Serial.print(ok1);
  Serial.print(", ");
  Serial.print(ok2);
  Serial.print(", ");
  Serial.println(ok3);

  return (ok1 && ok2 && ok3);
}

// ======================================================
// Full mission
// ======================================================
void runMission() {
  missionDone = false;

  oledShow("Started", "Running...");
  Serial.println("================================");
  Serial.println("Mission start");
  Serial.println("================================");

  // Step 1: approach middle light / pad area
  if (!approachMiddleLightAndPad()) {
    oledShow("Fail:", "Approach");
    STOP();
    missionDone = true;
    return;
  }

  STOP();
  delay(400);

  // Step 2: read pad color
  oledShow("Reading", "Color pad");
  PadColor pad = detectPadColor();

  // retry once if unknown
  if (pad == PAD_UNKNOWN) {
    delay(300);
    pad = detectPadColor();
  }

  if (pad == PAD_UNKNOWN) {
    oledShow("Color fail", "Unknown");
    Serial.println("Pad color unknown. Stopping.");
    STOP();
    missionDone = true;
    return;
  }

  // Step 3: side movement toward final parking light
  bool goRight = (pad == PAD_RED);
  if (!searchSideLightAndStop(goRight)) {
    oledShow("Fail:", "Side search");
    STOP();
    missionDone = true;
    return;
  }

  STOP();
  delay(300);

  // Step 4: final parallel + 8 cm
  if (!finalParkingAt8cm()) {
    oledShow("Fail:", "Final park");
    STOP();
    missionDone = true;
    return;
  }

  oledShow("Mission", "Complete");
  Serial.println("================================");
  Serial.println("Mission complete");
  Serial.println("================================");
  STOP();
  missionDone = true;
}

// ======================================================
// Voltage send from your code
// ======================================================
void sendVolt() {
  newV = analogRead(A0);
  if (newV != oldV) {
    if (!Serial3.available()) {
      Serial3.println(newV);
      Serial.println(newV);
    }
  }
  oldV = newV;
}

// ======================================================
// Setup
// ======================================================
void setup() {
  SERIAL.begin(115200);
  Serial3.begin(38400);

  // motor pins
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);

  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);

  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);

  // voltage
  pinMode(A0, INPUT);

  // sensors
  pinMode(LDR_LEFT, INPUT);
  pinMode(LDR_RIGHT, INPUT);

  pinMode(TRIG_LEFT, OUTPUT);
  pinMode(ECHO_LEFT, INPUT);
  pinMode(TRIG_RIGHT, OUTPUT);
  pinMode(ECHO_RIGHT, INPUT);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(sensorOut, INPUT);

  // TCS3200 frequency scaling = 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  // servos
  servo_pan.attach(48);
  servo_tilt.attach(47);
  servo_pan.write(pan);
  servo_tilt.write(tilt);

  // oled
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("AI Robot");
  display.display();

  STOP();

  Serial.println("System ready.");
  Serial.println("Type 's' in Serial Monitor to start mission.");
  oledShow("Ready", "Send: s");
}

// ======================================================
// Loop
// ======================================================
void loop() {
  // keep servo safe
  pan = constrain(pan, servo_min, servo_max);
  tilt = constrain(tilt, servo_min, servo_max);
  servo_pan.write(pan);
  servo_tilt.write(tilt);

  // print sensor data regularly for record
  if (millis() > (timeNow + 250)) {
    timeNow = millis();
    readAllSensors();
    printSensorLog();
    voltCount++;
  }

  if (voltCount >= 5) {
    voltCount = 0;
    sendVolt();
  }

  delay(5000);
  runMission();

  // // serial start
  // if (!missionStarted && Serial.available()) {
  //   char c = Serial.read();
  //   if (c == 's' || c == 'S') {
  //     missionStarted = true;
  //     runMission();
  //   }
  // }

  // optional reset command
  if (missionDone && Serial.available()) {
    char c = Serial.read();
    if (c == 'r' || c == 'R') {
      missionStarted = false;
      missionDone = false;
      STOP();
      oledShow("Reset", "Send: s");
      Serial.println("Reset complete. Type 's' to start again.");
    }
  }
}