#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// Creates the display object (declared here; defined in display.cpp)
extern Adafruit_SSD1306 display;

void setupDisplay();
void oledShowText(const String& text);