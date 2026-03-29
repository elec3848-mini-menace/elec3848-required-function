/**
 * @file behavior_align_side.cpp
 * @brief Side wall alignment behaviors.
 */
#include "behavior.h"
#include "behavior_internal.h"
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "oled.h"

enum AlignSideWallSubState {
    ALIGN_SIDE_INIT,
    ALIGN_SIDE_MEASURE,
    ALIGN_SIDE_CHECK_DIST,
    ALIGN_SIDE_MOVE_DELAY,
    ALIGN_SIDE_DONE
};

static AlignSideWallSubState currentAlignSideSubState = ALIGN_SIDE_INIT;
static float targetSideDistanceCM = 0;
static int currentSideDist = 0;
static unsigned long lastActionTime = 0;

static const char* directionName(int moveDirection) {
    if (moveDirection == moveLeft) {
        return "LEFT";
    }
    return "RIGHT";
}

static void startAlignWithSideWallCommon(RobotState targetState, float distanceCM) {
    Serial.print("Start side wall alignment to ");
    Serial.print(distanceCM);
    Serial.println("cm");

    currentRobotState = targetState;
    currentAlignSideSubState = ALIGN_SIDE_INIT;
    lastActionTime = millis();
    targetSideDistanceCM = distanceCM;
    oledShowText("Aligning Side", "Dist:---");
}

static void updateAligningSideWallCommon(UltrasonicSensorID sensorId, int moveWhenTooFar, int moveWhenTooClose) {
    const unsigned long currentMillis = millis();

    switch (currentAlignSideSubState) {
        case ALIGN_SIDE_INIT:
            currentAlignSideSubState = ALIGN_SIDE_MEASURE;
            lastActionTime = currentMillis;
            break;

        case ALIGN_SIDE_MEASURE:
            if (currentMillis - lastActionTime >= 100) {
                currentSideDist = getUltrasonicCM(sensorId);
                lastActionTime = currentMillis;
                currentAlignSideSubState = ALIGN_SIDE_CHECK_DIST;
            }
            break;

        case ALIGN_SIDE_CHECK_DIST:
            oledShowText("Aligning Side", "Dist:" + String(currentSideDist) + "cm");

            if (currentSideDist >= 300 || currentSideDist <= 0) {
                stopRobot();
                Serial.println("Invalid side ultrasonic reading, re-measuring...");
                currentAlignSideSubState = ALIGN_SIDE_MEASURE;
                lastActionTime = currentMillis;
                break;
            }

            if (abs(currentSideDist - targetSideDistanceCM) <= DISTANCE_TOLERANCE_CM) {
                stopRobot();
                Serial.println("Side wall alignment achieved.");
                currentAlignSideSubState = ALIGN_SIDE_DONE;
            } else if (currentSideDist > targetSideDistanceCM) {
                Serial.print("Too far from side wall, moving ");
                Serial.println(directionName(moveWhenTooFar));

                moveRobot(moveWhenTooFar, STRAFE_SPEED);
                currentAlignSideSubState = ALIGN_SIDE_MOVE_DELAY;
                lastActionTime = currentMillis;
            } else {
                Serial.print("Too close to side wall, moving ");
                Serial.println(directionName(moveWhenTooClose));

                moveRobot(moveWhenTooClose, STRAFE_SPEED);
                currentAlignSideSubState = ALIGN_SIDE_MOVE_DELAY;
                lastActionTime = currentMillis;
            }
            break;

        case ALIGN_SIDE_MOVE_DELAY:
            if (currentMillis - lastActionTime >= SIDE_ALIGN_MOVE_DURATION_MS) {
                stopRobot();
                currentAlignSideSubState = ALIGN_SIDE_MEASURE;
                lastActionTime = currentMillis;
            }
            break;

        case ALIGN_SIDE_DONE:
            currentRobotState = BEHAVIOR_COMPLETE;
            Serial.println("Side wall alignment complete.");
            oledShowText("Side Aligned!", "Dist: " + String(targetSideDistanceCM) + "cm");
            break;
    }
}

void startAlignWithSideWallRight(float distanceCM) {
    startAlignWithSideWallCommon(ALIGNING_SIDE_WALL_RIGHT, distanceCM);
}

void startAlignWithSideWallLeft(float distanceCM) {
    startAlignWithSideWallCommon(ALIGNING_SIDE_WALL_LEFT, distanceCM);
}

void updateAligningSideWallRight() {
    updateAligningSideWallCommon(US_SIDE_RIGHT, moveRight, moveLeft);
}

void updateAligningSideWallLeft() {
    updateAligningSideWallCommon(US_SIDE_LEFT, moveLeft, moveRight);
}
