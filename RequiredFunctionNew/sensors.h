#pragma once
#include <Arduino.h>

enum UltrasonicSensorID {
    US_FRONT_LEFT = 0,
    US_FRONT_RIGHT = 1,
    US_SIDE_RIGHT = 2
};

enum LightSensorID {
    LS_LEFT = 0,
    LS_RIGHT = 1
};

void setupSensors();
void calibrateLightSensor();
int getUltrasonicCM(UltrasonicSensorID sensor_id);

int getLightSensor(LightSensorID sensor_id);

void setupColorSensor();
void readRGB(int &r, int &g, int &b);
void readRGBAverage(int &r, int &g, int &b);
int detectFloorColor(); // Returns 0 for unknown, 1 for green, 2 for red