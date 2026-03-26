#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h> // Include SoftwareSerial library

extern Adafruit_SSD1306 display;
extern SoftwareSerial btSerial; // Declare the Bluetooth serial object

void setupDisplay();
void oledShowText(const String& line1, const String& line2 = "");