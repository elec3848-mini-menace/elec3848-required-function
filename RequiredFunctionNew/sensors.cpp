#include "sensors.h"
#include "config_pins.h"
#include <Arduino.h>

void setupSensors() {
	// Setup trigger pins as OUTPUT and echo pins as INPUT for ultrasonic sensors
	pinMode(LEFT_US_TRIG, OUTPUT);
	pinMode(LEFT_US_ECHO, INPUT);

	pinMode(RIGHT_US_TRIG, OUTPUT);
	pinMode(RIGHT_US_ECHO, INPUT);

	Serial.println("Sensors setup complete.");
}

int getUltrasonicCM(int sensor) {
	int trigPin = 0;
	int echoPin = 0;

	// Select the correct pins based on sensor index
	if (sensor == 0) { // Left sensor
		trigPin = LEFT_US_TRIG;
		echoPin = LEFT_US_ECHO;
	} else if (sensor == 1) { // Right sensor
		trigPin = RIGHT_US_TRIG;
		echoPin = RIGHT_US_ECHO;
	} else {
		// Handle invalid sensor index, return a large value to indicate error
		return 999;
	}

	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

	long duration = pulseIn(echoPin, HIGH, 50000);
	int cm = (duration / 2) / 29.1;

	if (cm <= 0 || cm > 250) {
		cm = 999;
	}
	return cm;
}