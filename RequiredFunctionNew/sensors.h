#pragma once
#include <Arduino.h>

void setupSensors();
int getUltrasonicCM(int sensor); // 0=left, 1=right