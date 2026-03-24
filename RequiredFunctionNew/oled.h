#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

void setupDisplay();
void oledShowText(const String& line1, const String& line2 = "");