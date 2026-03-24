#include "sensors.h"
#include "config_pins.h"

void setupSensors() {
  pinMode(BACK_LEFT_US_TRIG, OUTPUT);
  pinMode(BACK_LEFT_US_ECHO, INPUT);

  pinMode(BACK_RIGHT_US_TRIG, OUTPUT);
  pinMode(BACK_RIGHT_US_ECHO, INPUT);
}

int getUltrasonicCM(int sensor) {
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

  if (cm <= 0) cm = 999;
  return cm;
}