/**
 * @file behavior_post_turn.cpp
 * @brief Post-turn angle and front-distance alignment behaviors.
 */
#include "behavior.h"
#include "behavior_internal.h"
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "oled.h"

enum PostTurnFrontSubState {
    POST_TURN_FRONT_INIT,
    POST_TURN_FRONT_MEASURE_LEFT,
    POST_TURN_FRONT_MEASURE_RIGHT,
    POST_TURN_FRONT_CHECK,
    POST_TURN_FRONT_MOVE_DELAY,
    POST_TURN_FRONT_DONE
};

enum PostTurnAngleSubState {
    POST_TURN_ANGLE_INIT,
    POST_TURN_ANGLE_MEASURE_LEFT,
    POST_TURN_ANGLE_MEASURE_RIGHT,
    POST_TURN_ANGLE_CHECK,
    POST_TURN_ANGLE_TURN_DELAY,
    POST_TURN_ANGLE_DONE
};

static PostTurnFrontSubState currentPostTurnFrontSubState = POST_TURN_FRONT_INIT;
static PostTurnAngleSubState currentPostTurnAngleSubState = POST_TURN_ANGLE_INIT;

static float targetPostTurnFrontDistanceCM = 0;

static int frontLeftDist = 0;
static int frontRightDist = 0;
static int angleLeftDist = 0;
static int angleRightDist = 0;

static unsigned long lastPostTurnFrontActionTime = 0;
static unsigned long lastPostTurnAngleActionTime = 0;

void startAlignPostTurnFrontDistance(float targetDistanceCM) {
    Serial.print("Start post-turn front distance alignment to ");
    Serial.print(targetDistanceCM);
    Serial.println("cm");

    currentRobotState = ALIGNING_POST_TURN_FRONT_DISTANCE;
    currentPostTurnFrontSubState = POST_TURN_FRONT_INIT;
    targetPostTurnFrontDistanceCM = targetDistanceCM;
    lastPostTurnFrontActionTime = millis();

    oledShowText("Post-Turn Front", "Target:" + String(targetDistanceCM) + "cm");
}

void startAlignPostTurnAngle() {
    Serial.println("Start post-turn angle alignment");

    currentRobotState = ALIGNING_POST_TURN_ANGLE;
    currentPostTurnAngleSubState = POST_TURN_ANGLE_INIT;
    lastPostTurnAngleActionTime = millis();

    oledShowText("Post-Turn Angle", "L:--- R:---");
}

void updateAligningPostTurnFrontDistance() {
    const unsigned long currentMillis = millis();

    switch (currentPostTurnFrontSubState) {
        case POST_TURN_FRONT_INIT:
            stopRobot();
            currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_LEFT;
            lastPostTurnFrontActionTime = currentMillis;
            break;

        case POST_TURN_FRONT_MEASURE_LEFT:
            if (currentMillis - lastPostTurnFrontActionTime >= 60) {
                frontLeftDist = getUltrasonicCM(US_FRONT_LEFT);
                lastPostTurnFrontActionTime = currentMillis;
                currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_RIGHT;
            }
            break;

        case POST_TURN_FRONT_MEASURE_RIGHT:
            if (currentMillis - lastPostTurnFrontActionTime >= 60) {
                frontRightDist = getUltrasonicCM(US_FRONT_RIGHT);
                lastPostTurnFrontActionTime = currentMillis;
                currentPostTurnFrontSubState = POST_TURN_FRONT_CHECK;
            }
            break;

        case POST_TURN_FRONT_CHECK: {
            oledShowText("Post-Turn Front", "L:" + String(frontLeftDist) + " R:" + String(frontRightDist));

            if (frontLeftDist >= 300 || frontRightDist >= 300 || frontLeftDist <= 0 || frontRightDist <= 0) {
                stopRobot();
                Serial.println("Invalid post-turn front reading, re-measuring...");
                currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_LEFT;
                lastPostTurnFrontActionTime = currentMillis;
                break;
            }

            const int averageDist = (frontLeftDist + frontRightDist) / 2;

            if (abs(averageDist - targetPostTurnFrontDistanceCM) <= DISTANCE_TOLERANCE_CM) {
                stopRobot();
                Serial.println("Post-turn front distance achieved.");
                currentPostTurnFrontSubState = POST_TURN_FRONT_DONE;
            } else if (averageDist > targetPostTurnFrontDistanceCM) {
                Serial.println("Too far from post-turn front wall, moving FORWARD");
                moveRobot(moveForward, MAIN_SPEED);
                currentPostTurnFrontSubState = POST_TURN_FRONT_MOVE_DELAY;
                lastPostTurnFrontActionTime = currentMillis;
            } else {
                Serial.println("Too close to post-turn front wall, moving BACKWARD");
                moveRobot(moveBackward, MAIN_SPEED);
                currentPostTurnFrontSubState = POST_TURN_FRONT_MOVE_DELAY;
                lastPostTurnFrontActionTime = currentMillis;
            }
            break;
        }

        case POST_TURN_FRONT_MOVE_DELAY:
            if (currentMillis - lastPostTurnFrontActionTime >= POST_TURN_FRONT_MOVE_DURATION_MS) {
                stopRobot();
                currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_LEFT;
                lastPostTurnFrontActionTime = currentMillis;
            }
            break;

        case POST_TURN_FRONT_DONE:
            currentRobotState = BEHAVIOR_COMPLETE;
            oledShowText("Post-Turn Front", "Done");
            break;
    }
}

void updateAligningPostTurnAngle() {
    const unsigned long currentMillis = millis();

    switch (currentPostTurnAngleSubState) {
        case POST_TURN_ANGLE_INIT:
            stopRobot();
            currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_LEFT;
            lastPostTurnAngleActionTime = currentMillis;
            break;

        case POST_TURN_ANGLE_MEASURE_LEFT:
            if (currentMillis - lastPostTurnAngleActionTime >= 60) {
                angleLeftDist = getUltrasonicCM(US_FRONT_LEFT);
                lastPostTurnAngleActionTime = currentMillis;
                currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_RIGHT;
            }
            break;

        case POST_TURN_ANGLE_MEASURE_RIGHT:
            if (currentMillis - lastPostTurnAngleActionTime >= 60) {
                angleRightDist = getUltrasonicCM(US_FRONT_RIGHT);
                lastPostTurnAngleActionTime = currentMillis;
                currentPostTurnAngleSubState = POST_TURN_ANGLE_CHECK;
            }
            break;

        case POST_TURN_ANGLE_CHECK:
            oledShowText("Post-Turn Angle", "L:" + String(angleLeftDist) + " R:" + String(angleRightDist));

            if (angleLeftDist >= 300 || angleRightDist >= 300 || angleLeftDist <= 0 || angleRightDist <= 0) {
                stopRobot();
                Serial.println("Invalid post-turn angle reading, re-measuring...");
                currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_LEFT;
                lastPostTurnAngleActionTime = currentMillis;
                break;
            }

            if (abs(angleLeftDist - angleRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
                stopRobot();
                Serial.println("Post-turn angle alignment complete.");
                currentPostTurnAngleSubState = POST_TURN_ANGLE_DONE;
            } else if (angleLeftDist > angleRightDist) {
                Serial.println("Left > Right, turning RIGHT");
                moveRobot(moveClockwise, TURN_SPEED);
                currentPostTurnAngleSubState = POST_TURN_ANGLE_TURN_DELAY;
                lastPostTurnAngleActionTime = currentMillis;
            } else {
                Serial.println("Right > Left, turning LEFT");
                moveRobot(moveCounterclockwise, TURN_SPEED);
                currentPostTurnAngleSubState = POST_TURN_ANGLE_TURN_DELAY;
                lastPostTurnAngleActionTime = currentMillis;
            }
            break;

        case POST_TURN_ANGLE_TURN_DELAY:
            if (currentMillis - lastPostTurnAngleActionTime >= ALIGN_TURN_DURATION_MS) {
                stopRobot();
                delay(80);
                resetEncoderCounts();
                currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_LEFT;
                lastPostTurnAngleActionTime = currentMillis;
            }
            break;

        case POST_TURN_ANGLE_DONE:
            currentRobotState = BEHAVIOR_COMPLETE;
            oledShowText("Post-Turn Angle", "Done");
            break;
    }
}
