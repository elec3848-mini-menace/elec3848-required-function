/**
 * @file sensors.h
 * @brief Sensor interface for the ELEC3848 robot.
 *
 * Provides access to:
 *   - Four HC-SR04 ultrasonic sensors (2 front, 1 right side, 1 left side)
 *   - Two analogue ambient-light sensors
 *   - One TCS3200 RGB colour sensor for floor-colour detection
 */
#pragma once
#include <Arduino.h>

enum UltrasonicSensorID {
    US_FRONT_LEFT = 0,  ///< Front-left HC-SR04
    US_FRONT_RIGHT = 1, ///< Front-right HC-SR04
    US_SIDE_RIGHT = 2,  ///< Right-facing HC-SR04
    US_SIDE_LEFT = 3    ///< Left-facing HC-SR04
};

enum LightSensorID {
    LS_LEFT = 0,  ///< Left analogue light sensor
    LS_RIGHT = 1  ///< Right analogue light sensor
};

// Initialise all sensor GPIO pins and set TCS3200 frequency scaling to 20 %.
void setupSensors();

// Run a two-point calibration of the analogue light sensors.
// Place sensors under ambient light when prompted, then cover them.
void calibrateLightSensor();

// Trigger an HC-SR04 pulse and return the measured distance in cm.
// Returns 999 if the reading is out of range or invalid.
int getUltrasonicCM(UltrasonicSensorID sensor_id);

// Return the calibrated light intensity (0-100) for the specified sensor.
int getLightSensor(LightSensorID sensor_id);

// Read a single raw RGB sample from the TCS3200. Lower values = more light
// of that channel (raw period in microseconds from pulseIn).
void setupColorSensor();
void readRGB(int &r, int &g, int &b);

// Read three RGB samples and return their average (noise reduction).
void readRGBAverage(int &r, int &g, int &b);

// Detect the floor tile colour. Returns 1 = green, 2 = red, 0 = unknown.
int detectFloorColor();