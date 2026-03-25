#include "sensors.h"
#include "config_pins.h"
#include <Arduino.h>
#include "oled.h"

void setupSensors() {
	pinMode(LEFT_US_TRIG, OUTPUT);
	pinMode(LEFT_US_ECHO, INPUT);

	pinMode(RIGHT_US_TRIG, OUTPUT);
	pinMode(RIGHT_US_ECHO, INPUT);

    pinMode(RIGHT_US_TRIG_SIDE, OUTPUT);
    pinMode(RIGHT_US_ECHO_SIDE, INPUT);

    pinMode(LEFT_LIGHT_SENSOR, INPUT);
    pinMode(RIGHT_LIGHT_SENSOR, INPUT);

    pinMode(COLOR_S0, OUTPUT);
    pinMode(COLOR_S1, OUTPUT);
    pinMode(COLOR_S2, OUTPUT);
    pinMode(COLOR_S3, OUTPUT);
    pinMode(COLOR_OUT, INPUT);

    // TCS3200 frequency scaling = 20%
    digitalWrite(COLOR_S0, HIGH);
    digitalWrite(COLOR_S1, LOW);

	Serial.println("Sensors setup complete.");
}


//variables for light intensity to ADC reading equations 
int int_adc0, int_adc0_m, int_adc0_c;
int int_adc1, int_adc1_m, int_adc1_c;     
int int_left, int_right;

void calibrateLightSensor() {
    // measure the sensors reading at ambient light intensity  
    oledShowText("Calibration in progress, put the sensors under the light (~ 2 sec) ......");
    delay(2000);        // delay 5000 ms

    int_adc0=analogRead(LEFT_LIGHT_SENSOR);   // Left sensor at ambient light intensity
    int_adc1=analogRead(RIGHT_LIGHT_SENSOR);   // Right sensor at ambient light intensity
    Serial.print("Left : ");
    Serial.println(int_adc0);
    Serial.print("Right : ");
    Serial.println(int_adc1);
    delay(1000); 

    oledShowText("************ Put Fingers *****************");
    delay(5000);        // delay 5000 ms
    Serial.println("********* START Calibration **************");
    oledShowText("********* START Calibration **************");
    // measure the sensors reading at zero light intensity  
    int_adc0_c=analogRead(LEFT_LIGHT_SENSOR);   // Left sensor at zero light intensity
    int_adc1_c=analogRead(RIGHT_LIGHT_SENSOR);   // Right sensor at zero light intensity

    // calculate the slope of light intensity to ADC reading equations  
    int_adc0_m=(int_adc0-int_adc0_c)/100;
    int_adc1_m=(int_adc1-int_adc1_c)/100;
    delay(3000);     
    oledShowText("\n******** Completed! Remove your hands ********");
    oledShowText()
    delay(2000);        
}

int getUltrasonicCM(UltrasonicSensorID sensor_id) {
	int trigPin = 0;
	int echoPin = 0;

	// Select the correct pins based on sensor index
	switch (sensor_id) {
        case US_FRONT_LEFT:
            trigPin = LEFT_US_TRIG;
            echoPin = LEFT_US_ECHO;
            break;
        case US_FRONT_RIGHT:
            trigPin = RIGHT_US_TRIG;
            echoPin = RIGHT_US_ECHO;
            break;
        case US_SIDE_RIGHT:
            trigPin = RIGHT_US_TRIG_SIDE;
            echoPin = RIGHT_US_ECHO_SIDE;
            break;
        default:
            // Handle invalid sensor index, return a large value to indicate error
            return 999;
    }

	digitalWrite(trigPin, LOW);
	delayMicroseconds(2);
	digitalWrite(trigPin, HIGH);
	delayMicroseconds(10);
	digitalWrite(trigPin, LOW);

	long duration = pulseIn(echoPin, HIGH, 30000); // Timeout after 30ms (max distance ~5m)
	int cm = (duration / 2) / 29.1;

    // Filter out unrealistic readings
	if (cm <= 0 || cm > 250) {
		cm = 999; // Indicate out of range or error
	}
	return cm;
}

 // calculate the light intensity of the sensors
int getLightSensor(LightSensorID sensor_id) {
    switch (sensor_id) {
        case LS_LEFT:
            return (analogRead(LEFT_LIGHT_SENSOR)-int_adc0_c)/int_adc0_m;
        case LS_RIGHT:
            return (analogRead(RIGHT_LIGHT_SENSOR)-int_adc1_c)/int_adc1_m;
        default:
            return -1; // Error
    }
}

// Helper function to read raw RGB values (period of frequency, lower is more intense)
void readRGB(int &r, int &g, int &b) {
    // Read Red
    digitalWrite(COLOR_S2, LOW);
    digitalWrite(COLOR_S3, LOW);
    delay(COLOR_READ_DELAY_MS); // Allow sensor to stabilize
    r = pulseIn(COLOR_OUT, LOW); // Measure period, lower period = higher frequency = more light

    // Read Green
    digitalWrite(COLOR_S2, HIGH);
    digitalWrite(COLOR_S3, HIGH);
    delay(COLOR_READ_DELAY_MS);
    g = pulseIn(COLOR_OUT, LOW);

    // Read Blue
    digitalWrite(COLOR_S2, LOW);
    digitalWrite(COLOR_S3, HIGH);
    delay(COLOR_READ_DELAY_MS);
    b = pulseIn(COLOR_OUT, LOW);
}

void readRGBAverage(int &r, int &g, int &b) {
  int r1, g1, b1;
  int r2, g2, b2;
  int r3, g3, b3;

  readRGB(r1, g1, b1);
  delay(30);
  readRGB(r2, g2, b2);
  delay(30);
  readRGB(r3, g3, b3);

  r = (r1 + r2 + r3) / 3;
  g = (g1 + g2 + g3) / 3;
  b = (b1 + b2 + b3) / 3;
}

// Returns 0 for unknown, 1 for green, 2 for red
int detectFloorColor() {
    int r, g, b;
    readRGBAverage(r, g, b);

    Serial.print("Color detected - R:"); Serial.print(r);
    Serial.print(" G:"); Serial.print(g);
    Serial.print(" B:"); Serial.println(b);

    // Assuming lower pulseIn value means higher intensity
    if (r < g && r < b && r < 1000) {
        Serial.println("RED detected");
        return 2; // Red
    }
    else if (g < r && g < b && g < 1000) {
        Serial.println("GREEN detected");
        return 1; // Green
    }
    else {
        Serial.println("No clear red or green detected");
        return 0; // Unknown
    }
}