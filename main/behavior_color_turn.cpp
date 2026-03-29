/**
 * @file behavior_color_turn.cpp
 * @brief Floor-color detection and 90-degree turn behavior.
 */
#include "behavior.h"
#include "behavior_internal.h"
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "oled.h"

enum ColorTurnSubState {
    COLOR_TURN_INIT,
    COLOR_TURN_DETECT,
    COLOR_TURN_ACTION,
    COLOR_TURN_DELAY,
    COLOR_TURN_DONE
};

static ColorTurnSubState currentColorTurnSubState = COLOR_TURN_INIT;
static int detectedColor = 0;      // 0=unknown, 1=green, 2=red
static int lastDetectedColor = 0;  // 0=unknown, 1=green, 2=red
static long turnStartTicks = 0;
static unsigned long turnStartTime = 0;
static unsigned long lastActionTime = 0;

void startDetectColorAndTurn() {
    Serial.println("Start color detection and turn");
    currentRobotState = DETECTING_COLOR;
    currentColorTurnSubState = COLOR_TURN_INIT;
    lastActionTime = millis();
    detectedColor = 0;
    lastDetectedColor = 0;
    oledShowText("Detecting Color", "...");
}

int getLastDetectedColor() {
    return lastDetectedColor;
}

void updateDetectingColor() {
    const unsigned long currentMillis = millis();

    switch (currentColorTurnSubState) {
        case COLOR_TURN_INIT:
            stopRobot();
            Serial.println("Starting color detection...");
            currentColorTurnSubState = COLOR_TURN_DETECT;
            lastActionTime = currentMillis;
            break;

        case COLOR_TURN_DETECT:
            if (currentMillis - lastActionTime >= 100) {
                detectedColor = detectFloorColor();
                if (detectedColor != 0) {
                    lastDetectedColor = detectedColor;
                    currentColorTurnSubState = COLOR_TURN_ACTION;
                    lastActionTime = currentMillis;
                } else {
                    Serial.println("No clear color, re-detecting...");
                    lastActionTime = currentMillis;
                }
            }
            break;

        case COLOR_TURN_ACTION:
            resetEncoderCounts();
            turnStartTicks = getAverageAbsoluteEncoderCount();
            turnStartTime = currentMillis;

            if (detectedColor == 1) {
                Serial.println("Green detected, turning LEFT (Counterclockwise)");
                moveRobot(moveCounterclockwise, TURN_SPEED);
            } else if (detectedColor == 2) {
                Serial.println("Red detected, turning RIGHT (Clockwise)");
                moveRobot(moveClockwise, TURN_SPEED);
            } else {
                Serial.println("Unknown color, stopping.");
                stopRobot();
                currentColorTurnSubState = COLOR_TURN_DONE;
                break;
            }

            oledShowText("Color: " + String(detectedColor == 1 ? "Green" : "Red"), "Turning...");
            currentColorTurnSubState = COLOR_TURN_DELAY;
            break;

        case COLOR_TURN_DELAY: {
            const long currentTicks = getAverageAbsoluteEncoderCount();

            if (currentTicks - turnStartTicks >= TURN_90_DEGREE_ENCODER_TICKS) {
                stopRobot();
                delay(200);
                resetEncoderCounts();
                Serial.println("Encoder-based 90 degree turn complete.");
                currentColorTurnSubState = COLOR_TURN_DONE;
            } else if (currentMillis - turnStartTime >= TURN_90_TIMEOUT_MS) {
                stopRobot();
                delay(200);
                resetEncoderCounts();
                Serial.println("Turn timeout reached.");
                currentColorTurnSubState = COLOR_TURN_DONE;
            }
            break;
        }

        case COLOR_TURN_DONE:
            currentRobotState = BEHAVIOR_COMPLETE;
            Serial.println("Color detection and turn behavior complete.");
            oledShowText("Turn Complete!", "Next Step...");
            break;
    }
}
