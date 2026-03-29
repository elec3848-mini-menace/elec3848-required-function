/**
 * @file behavior_align_wall.cpp
 * @brief Front wall alignment behavior using dual ultrasonic sensors.
 */
#include "behavior.h"
#include "behavior_internal.h"
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "oled.h"

enum AlignWallSubState {
    ALIGN_INIT,
    ALIGN_MEASURE_LEFT,
    ALIGN_MEASURE_RIGHT,
    ALIGN_CHECK,
    ALIGN_TURN_DELAY,
    ALIGN_CONFIRM_MEASURE_LEFT,
    ALIGN_CONFIRM_MEASURE_RIGHT,
    ALIGN_MOVING_TO_TARGET_DIST,
    ALIGN_MOVING_TO_TARGET_DIST_MEASURE_LEFT,
    ALIGN_MOVING_TO_TARGET_DIST_MEASURE_RIGHT,
    ALIGN_DONE
};

static AlignWallSubState currentAlignSubState = ALIGN_INIT;
static unsigned long lastActionTime = 0;
static int currentLeftDist = 0;
static int currentRightDist = 0;
static int alignmentConfirmationCount = 0;
static int invalidFrontReadingCount = 0;
static unsigned long invalidFrontStartTime = 0;

void startAlignWithWallUsingUltrasonic() {
    Serial.println("Start front wall alignment");
    currentRobotState = ALIGNING_WALL;
    currentAlignSubState = ALIGN_INIT;
    lastActionTime = millis();
    alignmentConfirmationCount = 0;
    invalidFrontReadingCount = 0;
    invalidFrontStartTime = 0;
    oledShowText("Aligning Front", "L:--- R:---");
}

void updateAligningWall() {
    const unsigned long currentMillis = millis();

    switch (currentAlignSubState) {
        case ALIGN_INIT:
            currentAlignSubState = ALIGN_MEASURE_LEFT;
            lastActionTime = currentMillis;
            break;

        case ALIGN_MEASURE_LEFT:
            if (currentMillis - lastActionTime >= 80) {
                currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
                lastActionTime = currentMillis;
                currentAlignSubState = ALIGN_MEASURE_RIGHT;
            }
            break;

        case ALIGN_MEASURE_RIGHT:
            if (currentMillis - lastActionTime >= 80) {
                currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
                lastActionTime = currentMillis;
                currentAlignSubState = ALIGN_CHECK;
            }
            break;

        case ALIGN_CHECK:
            oledShowText("Aligning Front", "L:" + String(currentLeftDist) + " R:" + String(currentRightDist));

            if (currentLeftDist >= 300 || currentRightDist >= 300 || currentLeftDist <= 0 || currentRightDist <= 0) {
                stopRobot();

                if (invalidFrontReadingCount == 0) {
                    invalidFrontStartTime = currentMillis;
                }

                invalidFrontReadingCount++;
                Serial.print("Invalid front ultrasonic reading. Count = ");
                Serial.println(invalidFrontReadingCount);
                oledShowText("US Invalid", "L:" + String(currentLeftDist) + " R:" + String(currentRightDist));

                if (invalidFrontReadingCount >= ULTRASONIC_999_THRESHOLD ||
                    (currentMillis - invalidFrontStartTime) >= ULTRASONIC_999_TIMEOUT_MS) {
                    Serial.println("Ultrasonic stuck at 999 too long. Skipping to next step.");
                    oledShowText("US Skip", "Proceed Next");
                    currentAlignSubState = ALIGN_DONE;
                } else {
                    currentAlignSubState = ALIGN_MEASURE_LEFT;
                    lastActionTime = currentMillis;
                }
                break;
            }

            invalidFrontReadingCount = 0;
            invalidFrontStartTime = 0;

            if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
                alignmentConfirmationCount = 0;
                currentAlignSubState = ALIGN_CONFIRM_MEASURE_LEFT;
                lastActionTime = currentMillis;
            } else if (currentLeftDist > currentRightDist) {
                Serial.println("Left > Right, turning RIGHT");
                moveRobot(moveClockwise, TURN_SPEED);
                currentAlignSubState = ALIGN_TURN_DELAY;
                lastActionTime = currentMillis;
            } else {
                Serial.println("Right > Left, turning LEFT");
                moveRobot(moveCounterclockwise, TURN_SPEED);
                currentAlignSubState = ALIGN_TURN_DELAY;
                lastActionTime = currentMillis;
            }
            break;

        case ALIGN_TURN_DELAY:
            if (currentMillis - lastActionTime >= ALIGN_TURN_DURATION_MS) {
                stopRobot();
                currentAlignSubState = ALIGN_MEASURE_LEFT;
                lastActionTime = currentMillis;
            }
            break;

        case ALIGN_CONFIRM_MEASURE_LEFT:
            oledShowText("Confirming Align", "L:--- R:" + String(currentRightDist));
            if (currentMillis - lastActionTime >= 50) {
                currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
                lastActionTime = currentMillis;
                currentAlignSubState = ALIGN_CONFIRM_MEASURE_RIGHT;
            }
            break;

        case ALIGN_CONFIRM_MEASURE_RIGHT:
            oledShowText("Confirming Align", "L:" + String(currentLeftDist) + " R:---");
            if (currentMillis - lastActionTime >= 50) {
                currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
                lastActionTime = currentMillis;

                if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
                    alignmentConfirmationCount++;
                    Serial.print("Confirmed count: ");
                    Serial.println(alignmentConfirmationCount);

                    if (alignmentConfirmationCount >= ALIGNMENT_CONFIRMATION_THRESHOLD) {
                        stopRobot();
                        Serial.println("Front wall alignment confirmed and stable.");
                        currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST;
                        lastActionTime = currentMillis;
                        oledShowText("Alignment Confirmed!", "Moving to " + String(TARGET_WALL_DISTANCE_CM) + "cm");
                    } else {
                        currentAlignSubState = ALIGN_CONFIRM_MEASURE_LEFT;
                    }
                } else {
                    Serial.println("Alignment lost during confirmation, re-aligning.");
                    alignmentConfirmationCount = 0;
                    currentAlignSubState = ALIGN_MEASURE_LEFT;
                }
            }
            break;

        case ALIGN_MOVING_TO_TARGET_DIST:
            moveRobot(moveForward, MAIN_SPEED);
            Serial.print("Moving to target distance: ");
            Serial.print(TARGET_WALL_DISTANCE_CM);
            Serial.println("cm.");
            currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST_MEASURE_LEFT;
            lastActionTime = currentMillis;
            break;

        case ALIGN_MOVING_TO_TARGET_DIST_MEASURE_LEFT:
            if (currentMillis - lastActionTime >= 10) {
                currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
                lastActionTime = currentMillis;
                currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST_MEASURE_RIGHT;
            }
            break;

        case ALIGN_MOVING_TO_TARGET_DIST_MEASURE_RIGHT:
            if (currentMillis - lastActionTime >= 10) {
                currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
                lastActionTime = currentMillis;

                const bool leftInvalid = (currentLeftDist >= 300 || currentLeftDist <= 0);
                const bool rightInvalid = (currentRightDist >= 300 || currentRightDist <= 0);

                if (leftInvalid || rightInvalid) {
                    stopRobot();

                    if (invalidFrontReadingCount == 0) {
                        invalidFrontStartTime = currentMillis;
                    }

                    invalidFrontReadingCount++;
                    Serial.print("Invalid while moving to target. Count = ");
                    Serial.println(invalidFrontReadingCount);

                    if (invalidFrontReadingCount >= ULTRASONIC_999_THRESHOLD ||
                        (currentMillis - invalidFrontStartTime) >= ULTRASONIC_999_TIMEOUT_MS) {
                        Serial.println("Ultrasonic stuck at 999 too long while moving. Skipping to next step.");
                        oledShowText("US Skip", "Proceed Next");
                        currentAlignSubState = ALIGN_DONE;
                    } else {
                        currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST;
                        lastActionTime = currentMillis;
                    }
                    break;
                }

                invalidFrontReadingCount = 0;
                invalidFrontStartTime = 0;

                const int averageDist = (currentLeftDist + currentRightDist) / 2;
                oledShowText("To " + String(TARGET_WALL_DISTANCE_CM) + "cm", "Avg:" + String(averageDist) + "cm");

                if (averageDist <= TARGET_WALL_DISTANCE_CM + DISTANCE_TOLERANCE_CM) {
                    stopRobot();
                    Serial.print("Reached target distance of ");
                    Serial.print(TARGET_WALL_DISTANCE_CM);
                    Serial.println("cm.");
                    currentAlignSubState = ALIGN_DONE;
                } else {
                    currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST_MEASURE_LEFT;
                }
            }
            break;

        case ALIGN_DONE:
            currentRobotState = BEHAVIOR_COMPLETE;
            Serial.println("Front wall alignment and distance adjustment complete.");
            oledShowText("Front Aligned!", "Dist: " + String(TARGET_WALL_DISTANCE_CM) + "cm");
            break;
    }
}
