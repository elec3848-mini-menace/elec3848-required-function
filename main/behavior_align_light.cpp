/**
 * @file behavior_align_light.cpp
 * @brief Light-source centering behavior using dual analogue sensors.
 */
#include "behavior.h"
#include "behavior_internal.h"
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "oled.h"

enum AlignLightSubState {
    ALIGN_LIGHT_INIT,
    ALIGN_LIGHT_MEASURE,
    ALIGN_LIGHT_CHECK,
    ALIGN_LIGHT_MOVE_DELAY,
    ALIGN_LIGHT_DONE
};

static AlignLightSubState currentAlignLightSubState = ALIGN_LIGHT_INIT;
static int currentLeftLight = 0;
static int currentRightLight = 0;
static int stableLightCount = 0;
static unsigned long lastActionTime = 0;

void startAlignToLightSource() {
    Serial.println("Start light source alignment");
    currentRobotState = ALIGNING_TO_LIGHT_SOURCE;
    currentAlignLightSubState = ALIGN_LIGHT_INIT;
    lastActionTime = millis();
    stableLightCount = 0;
    oledShowText("Aligning Light", "L:--- R:---");
}

void updateAligningToLightSource() {
    const unsigned long currentMillis = millis();

    switch (currentAlignLightSubState) {
        case ALIGN_LIGHT_INIT:
            currentAlignLightSubState = ALIGN_LIGHT_MEASURE;
            lastActionTime = currentMillis;
            break;

        case ALIGN_LIGHT_MEASURE:
            if (currentMillis - lastActionTime >= 50) {
                currentLeftLight = getLightSensor(LS_LEFT);
                currentRightLight = getLightSensor(LS_RIGHT);
                lastActionTime = currentMillis;
                currentAlignLightSubState = ALIGN_LIGHT_CHECK;
            }
            break;

        case ALIGN_LIGHT_CHECK:
            oledShowText("Aligning Light", "L:" + String(currentLeftLight) + " R:" + String(currentRightLight));

            if (abs(currentLeftLight - currentRightLight) <= LIGHT_SENSOR_ALIGN_THRESHOLD) {
                stableLightCount++;
                if (stableLightCount >= LIGHT_SENSOR_STABLE_COUNT_THRESHOLD) {
                    stopRobot();
                    Serial.println("Light source alignment achieved.");
                    currentAlignLightSubState = ALIGN_LIGHT_DONE;
                } else {
                    currentAlignLightSubState = ALIGN_LIGHT_MEASURE;
                    lastActionTime = currentMillis;
                }
            } else if (currentLeftLight > currentRightLight) {
                Serial.println("Left light brighter, moving LEFT");
                moveRobot(moveLeft, STRAFE_SPEED);
                currentAlignLightSubState = ALIGN_LIGHT_MOVE_DELAY;
                lastActionTime = currentMillis;
                stableLightCount = 0;
            } else {
                Serial.println("Right light brighter, moving RIGHT");
                moveRobot(moveRight, STRAFE_SPEED);
                currentAlignLightSubState = ALIGN_LIGHT_MOVE_DELAY;
                lastActionTime = currentMillis;
                stableLightCount = 0;
            }
            break;

        case ALIGN_LIGHT_MOVE_DELAY:
            if (currentMillis - lastActionTime >= LIGHT_SENSOR_MOVE_DURATION_MS) {
                stopRobot();
                currentAlignLightSubState = ALIGN_LIGHT_MEASURE;
                lastActionTime = currentMillis;
            }
            break;

        case ALIGN_LIGHT_DONE:
            currentRobotState = BEHAVIOR_COMPLETE;
            Serial.println("Light source alignment complete.");
            oledShowText("Light Aligned!", "Center.");
            break;
    }
}
