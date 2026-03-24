#include <Arduino.h>

#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "behavior.h"
#include "oled.h"

bool missionDone = false;
unsigned long lastMotorSyncTime = 0;
const unsigned long MOTOR_SYNC_INTERVAL = 1;

void setup() {
  Serial.begin(115200);

  setupMotor();
  setupSensors();

  setupDisplay();

  oledShowText("Ready");
  stopRobot();
  delay(1000);
}

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