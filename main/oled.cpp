#include "oled.h"
#include "config_pins.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <SoftwareSerial.h>

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
SoftwareSerial btSerial(BT_RX_PIN, BT_TX_PIN);

void setupDisplay() {
    // Initialize OLED
    if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println("SSD1306 allocation failed");
        for (;;)
        ;
    }
    display.display();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("OLED Ready");
    display.display();
    delay(500);

    // Initialize Bluetooth serial
    btSerial.begin(BT_BAUD_RATE);
    delay(100); // Give Bluetooth module a moment to initialize
    btSerial.println("Bluetooth Ready"); // Send initial message
    Serial.println("Bluetooth serial initialized.");
}

void oledShowText(const String& line1, const String& line2) {
    // Display on OLED
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);
    display.setCursor(0, 0);

    display.println(line1);

    if (line2 != "") {
        display.println(line2);
    }
    display.display();

    // Send over Bluetooth
    btSerial.println(line1);
    btSerial.println(line2);
}