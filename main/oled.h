/**
 * @file oled.h
 * @brief SSD1306 128x32 OLED display helpers.
 */
#pragma once
#include <Arduino.h>
#include <Adafruit_SSD1306.h>

extern Adafruit_SSD1306 display;

// Initialise the SSD1306 display. Halts on failure.
void setupDisplay();

// Clear the display and print up to two lines of text.
void oledShowText(const String& line1, const String& line2 = "");