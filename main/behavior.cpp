#include "behavior.h"
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "oled.h" // Include oled for display updates

// Global state variables for behaviors
RobotState currentRobotState = IDLE;

// Variables for moveForwardDistanceCM state machine
static float targetDistanceCM = 0;
static long targetEncoderTicks = 0;
// For skipping ultrasonic alignment if sensor keeps returning 999
static int invalidFrontReadingCount = 0;
static unsigned long invalidFrontStartTime = 0;
static long turnStartTicks = 0;
static unsigned long turnStartTime = 0;

enum PostTurnAngleSubState {
    POST_TURN_ANGLE_INIT,
    POST_TURN_ANGLE_MEASURE_LEFT,
    POST_TURN_ANGLE_MEASURE_RIGHT,
    POST_TURN_ANGLE_CHECK,
    POST_TURN_ANGLE_TURN_DELAY,
    POST_TURN_ANGLE_DONE
};

static PostTurnAngleSubState currentPostTurnAngleSubState = POST_TURN_ANGLE_INIT;

// Variables for alignWithWallUsingUltrasonic state machine (front wall)
enum AlignWallSubState {
    ALIGN_INIT,
    ALIGN_MEASURE_LEFT,
    ALIGN_MEASURE_RIGHT,
    ALIGN_CHECK,
    ALIGN_TURN_LEFT,
    ALIGN_TURN_RIGHT,
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

// Variables for alignWithSideWall state machine
enum AlignSideWallSubState {
    ALIGN_SIDE_INIT,
    ALIGN_SIDE_MEASURE,
    ALIGN_SIDE_CHECK_DIST,
    ALIGN_SIDE_MOVE_LEFT,
    ALIGN_SIDE_MOVE_RIGHT,
    ALIGN_SIDE_MOVE_DELAY,
    ALIGN_SIDE_DONE
};
static AlignSideWallSubState currentAlignSideSubState = ALIGN_SIDE_INIT;
static float targetSideDistanceCM = 0;
static int currentSideDist = 0;

// Variables for alignToLightSource state machine
enum AlignLightSubState {
    ALIGN_LIGHT_INIT,
    ALIGN_LIGHT_MEASURE,
    ALIGN_LIGHT_CHECK,
    ALIGN_LIGHT_MOVE_LEFT,
    ALIGN_LIGHT_MOVE_RIGHT,
    ALIGN_LIGHT_MOVE_DELAY,
    ALIGN_LIGHT_CONFIRM,
    ALIGN_LIGHT_DONE
};
static AlignLightSubState currentAlignLightSubState = ALIGN_LIGHT_INIT;
static int currentLeftLight = 0;
static int currentRightLight = 0;
static int stableLightCount = 0;

// Variables for detectColorAndTurn state machine
enum ColorTurnSubState {
    COLOR_TURN_INIT,
    COLOR_TURN_DETECT,
    COLOR_TURN_ACTION,
    COLOR_TURN_DELAY,
    COLOR_TURN_DONE
};
static ColorTurnSubState currentColorTurnSubState = COLOR_TURN_INIT;
static int detectedColor = 0; // 0=unknown, 1=green, 2=red

// Remember the last detected color so the .ino file can choose left/right side sensor
static int lastDetectedColor = 0; // 0=unknown, 1=green, 2=red

// Variables for post-turn front distance adjustment
enum PostTurnFrontSubState {
    POST_TURN_FRONT_INIT,
    POST_TURN_FRONT_MEASURE_LEFT,
    POST_TURN_FRONT_MEASURE_RIGHT,
    POST_TURN_FRONT_CHECK,
    POST_TURN_FRONT_MOVE_FORWARD,
    POST_TURN_FRONT_MOVE_BACKWARD,
    POST_TURN_FRONT_MOVE_DELAY,
    POST_TURN_FRONT_DONE
};

static PostTurnFrontSubState currentPostTurnFrontSubState = POST_TURN_FRONT_INIT;
static float targetPostTurnFrontDistanceCM = 0;

void setupBehaviors() {
    currentRobotState = IDLE;
    Serial.println("Behaviors setup complete.");
}

void startMoveForwardDistanceCM(float distanceCM) {
    targetDistanceCM = distanceCM;
    targetEncoderTicks = (long)(distanceCM * TICKS_PER_CM);
    resetEncoderCounts();
    moveRobot(moveForward, MAIN_SPEED);
    currentRobotState = MOVING_FORWARD_DISTANCE;
    Serial.print("Starting move forward: ");
    Serial.print(distanceCM);
    Serial.println(" cm");
    oledShowText("Moving " + String(distanceCM) + "cm", "Enc: 0/" + String(targetEncoderTicks));
}

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

// Function to start aligning with the side wall right
void startAlignWithSideWallRight(float distanceCM) {
    Serial.print("Start side wall alignment to ");
    Serial.print(distanceCM);
    Serial.println("cm");
    currentRobotState = ALIGNING_SIDE_WALL_RIGHT;
    currentAlignSideSubState = ALIGN_SIDE_INIT;
    lastActionTime = millis();
    targetSideDistanceCM = distanceCM;
    oledShowText("Aligning Side", "Dist:---");
}

// Function to start aligning with the side wall left
void startAlignWithSideWallLeft(float distanceCM) {
    Serial.print("Start side wall alignment to ");
    Serial.print(distanceCM);
    Serial.println("cm");
    currentRobotState = ALIGNING_SIDE_WALL_LEFT;
    currentAlignSideSubState = ALIGN_SIDE_INIT;
    lastActionTime = millis();
    targetSideDistanceCM = distanceCM;
    oledShowText("Aligning Side", "Dist:---");
}


// Function to start aligning to a light source
void startAlignToLightSource() {
    Serial.println("Start light source alignment");
    currentRobotState = ALIGNING_TO_LIGHT_SOURCE;
    currentAlignLightSubState = ALIGN_LIGHT_INIT;
    lastActionTime = millis();
    stableLightCount = 0;
    oledShowText("Aligning Light", "L:--- R:---");
}

// Function to start detecting color and turning
void startDetectColorAndTurn() {
    Serial.println("Start color detection and turn");
    currentRobotState = DETECTING_COLOR;
    currentColorTurnSubState = COLOR_TURN_INIT;
    lastActionTime = millis();
    detectedColor = 0;
    lastDetectedColor = 0;
    oledShowText("Detecting Color", "...");
}

void startAlignPostTurnFrontDistance(float targetDistanceCM) {
    Serial.print("Start post-turn front distance alignment to ");
    Serial.print(targetDistanceCM);
    Serial.println("cm");

    currentRobotState = ALIGNING_POST_TURN_FRONT_DISTANCE;
    currentPostTurnFrontSubState = POST_TURN_FRONT_INIT;
    targetPostTurnFrontDistanceCM = targetDistanceCM;
    lastActionTime = millis();

    oledShowText("Post-Turn Front", "Target:" + String(targetDistanceCM) + "cm");
}

void startAlignPostTurnAngle() {
    Serial.println("Start post-turn angle alignment");

    currentRobotState = ALIGNING_POST_TURN_ANGLE;
    currentPostTurnAngleSubState = POST_TURN_ANGLE_INIT;
    lastActionTime = millis();

    oledShowText("Post-Turn Angle", "L:--- R:---");
}


int getLastDetectedColor() {
    return lastDetectedColor;
}

void updateBehaviors() {
    unsigned long currentMillis = millis();

    switch (currentRobotState) {
        case IDLE:
			// Robot is idle, waiting for a command
			break;

        case MOVING_FORWARD_DISTANCE:
			// Update OLED with current encoder counts
			oledShowText("Moving " + String(targetDistanceCM) + "cm",
						"Enc: " + String(getAverageEncoderCount()) + "/" + String(targetEncoderTicks));

			if (getAverageEncoderCount() >= targetEncoderTicks) {
				stopRobot();
				currentRobotState = BEHAVIOR_COMPLETE;
				Serial.println("Move forward complete.");
				oledShowText("Moved " + String(targetDistanceCM) + "cm", "Done.");
			}
			break;

        case ALIGNING_WALL: // Front wall alignment
			switch (currentAlignSubState) {
				case ALIGN_INIT:
					// Transition to first measurement
					currentAlignSubState = ALIGN_MEASURE_LEFT;
					lastActionTime = currentMillis; // Start timer for measurement
					break;

				case ALIGN_MEASURE_LEFT:
					if (currentMillis - lastActionTime >= 80) { // Delay before taking left ultrasonic reading
						currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
						lastActionTime = currentMillis; // Reset timer for next action
						currentAlignSubState = ALIGN_MEASURE_RIGHT;
					}
					break;

				case ALIGN_MEASURE_RIGHT:
					if (currentMillis - lastActionTime >= 80) { // Delay before taking right ultrasonic reading
						currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
						lastActionTime = currentMillis; // Reset timer for next action
						currentAlignSubState = ALIGN_CHECK;
					}
					break;

				case ALIGN_CHECK:
                    oledShowText("Aligning Front", "L:" + String(currentLeftDist) + " R:" + String(currentRightDist));

                    // Check for invalid ultrasonic readings (out of range or 0)
                    if (currentLeftDist >= 300 || currentRightDist >= 300 || currentLeftDist <= 0 || currentRightDist <= 0) {
                        stopRobot();

                        if (invalidFrontReadingCount == 0) {
                            invalidFrontStartTime = currentMillis;
                        }

                        invalidFrontReadingCount++;

                        Serial.print("Invalid front ultrasonic reading. Count = ");
                        Serial.println(invalidFrontReadingCount);

                        oledShowText("US Invalid",
                                    "L:" + String(currentLeftDist) + " R:" + String(currentRightDist));

                        if (invalidFrontReadingCount >= ULTRASONIC_999_THRESHOLD ||
                            (currentMillis - invalidFrontStartTime) >= ULTRASONIC_999_TIMEOUT_MS) {
                            Serial.println("Ultrasonic stuck at 999 too long. Skipping to next step.");
                            oledShowText("US Skip", "Proceed Next");
                            currentAlignSubState = ALIGN_DONE;
                        } else {
                            Serial.println("Invalid front ultrasonic reading, re-measuring...");
                            currentAlignSubState = ALIGN_MEASURE_LEFT;
                            lastActionTime = currentMillis;
                        }
                        break;
                    } else {
                        invalidFrontReadingCount = 0;
                        invalidFrontStartTime = 0;
                    }

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
						currentAlignSubState = ALIGN_MEASURE_LEFT; // Go back to measure after turn
						lastActionTime = currentMillis; // Delay before next measurement
					}
					break;

				case ALIGN_CONFIRM_MEASURE_LEFT:
					oledShowText("Confirming Align", "L:--- R:" + String(currentRightDist));

					if (currentMillis - lastActionTime >= 50) {   // wait before left measurement
						currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
						lastActionTime = currentMillis;             // start timer for right measurement
						currentAlignSubState = ALIGN_CONFIRM_MEASURE_RIGHT;
					}
					break;

				case ALIGN_CONFIRM_MEASURE_RIGHT:
					oledShowText("Confirming Align", "L:" + String(currentLeftDist) + " R:---");

					if (currentMillis - lastActionTime >= 50) {   // wait before right measurement
						currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
						lastActionTime = currentMillis;             // timer for next confirmation cycle

						if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
							alignmentConfirmationCount++;
							Serial.print("Confirmed count: "); Serial.println(alignmentConfirmationCount);

							if (alignmentConfirmationCount >= ALIGNMENT_CONFIRMATION_THRESHOLD) {
								stopRobot();
								Serial.println("Front wall alignment confirmed and stable.");
								currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST;
								lastActionTime = currentMillis;
								oledShowText("Alignment Confirmed!",
											"Moving to " + String(TARGET_WALL_DISTANCE_CM) + "cm");
							} else {
								// keep confirming: go measure-left again
								currentAlignSubState = ALIGN_CONFIRM_MEASURE_LEFT;
							}
						} else {
							Serial.println("Alignment lost during confirmation, re-aligning.");
							alignmentConfirmationCount = 0;
							currentAlignSubState = ALIGN_MEASURE_LEFT; // or ALIGN_MEASURE_LEFT stage
						}
					}
					break;

				case ALIGN_MOVING_TO_TARGET_DIST:
                    moveRobot(moveForward, MAIN_SPEED);
                    Serial.print("Moving to target distance: ");
                    Serial.print(TARGET_WALL_DISTANCE_CM);
                    Serial.println("cm.");

                    // Kick off first left measurement
                    currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST_MEASURE_LEFT;
                    lastActionTime = currentMillis; // start timer for left
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

                        bool leftInvalid  = (currentLeftDist  >= 300 || currentLeftDist  <= 0);
                        bool rightInvalid = (currentRightDist >= 300 || currentRightDist <= 0);

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
                        } else {
                            invalidFrontReadingCount = 0;
                            invalidFrontStartTime = 0;
                        }

                        int averageDist = (currentLeftDist + currentRightDist) / 2;
                        oledShowText("To " + String(TARGET_WALL_DISTANCE_CM) + "cm",
                                    "Avg:" + String(averageDist) + "cm");

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
			break;

        case ALIGNING_SIDE_WALL_RIGHT:
            switch (currentAlignSideSubState) {
                case ALIGN_SIDE_INIT:
                    currentAlignSideSubState = ALIGN_SIDE_MEASURE;
                    lastActionTime = currentMillis;
                    break;

                case ALIGN_SIDE_MEASURE:
                    if (currentMillis - lastActionTime >= 100) { // Delay before taking reading
                        currentSideDist = getUltrasonicCM(US_SIDE_RIGHT);
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
                    } else if (currentSideDist > targetSideDistanceCM) { // Too far from wall, move right
                        Serial.println("Too far from side wall, moving RIGHT");
                        moveRobot(moveRight, STRAFE_SPEED);
                        currentAlignSideSubState = ALIGN_SIDE_MOVE_DELAY;
                        lastActionTime = currentMillis;
                    } else { // Too close to wall, move left
                        Serial.println("Too close to side wall, moving LEFT");
                        moveRobot(moveLeft, STRAFE_SPEED);
                        currentAlignSideSubState = ALIGN_SIDE_MOVE_DELAY;
                        lastActionTime = currentMillis;
                    }
                    break;

                case ALIGN_SIDE_MOVE_DELAY:
                    if (currentMillis - lastActionTime >= SIDE_ALIGN_MOVE_DURATION_MS) {
                        stopRobot();
                        currentAlignSideSubState = ALIGN_SIDE_MEASURE; // Re-measure after move
                        lastActionTime = currentMillis;
                    }
                    break;

                case ALIGN_SIDE_DONE:
                    currentRobotState = BEHAVIOR_COMPLETE;
                    Serial.println("Side wall alignment complete.");
                    oledShowText("Side Aligned!", "Dist: " + String(targetSideDistanceCM) + "cm");
                    break;
            }
            break;

        case ALIGNING_SIDE_WALL_LEFT:
            switch (currentAlignSideSubState) {
                case ALIGN_SIDE_INIT:
                    currentAlignSideSubState = ALIGN_SIDE_MEASURE;
                    lastActionTime = currentMillis;
                    break;

                case ALIGN_SIDE_MEASURE:
                    if (currentMillis - lastActionTime >= 100) { // Delay before taking reading
                        currentSideDist = getUltrasonicCM(US_SIDE_LEFT);
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
                    } else if (currentSideDist > targetSideDistanceCM) { // Too far from wall, move right
                        Serial.println("Too far from side wall, moving LEFT");
                        moveRobot(moveLeft, STRAFE_SPEED);
                        currentAlignSideSubState = ALIGN_SIDE_MOVE_DELAY;
                        lastActionTime = currentMillis;
                    } else { // Too close to wall, move left
                        Serial.println("Too close to side wall, moving RIGHT");
                        moveRobot(moveRight, STRAFE_SPEED);
                        currentAlignSideSubState = ALIGN_SIDE_MOVE_DELAY;
                        lastActionTime = currentMillis;
                    }
                    break;

                case ALIGN_SIDE_MOVE_DELAY:
                    if (currentMillis - lastActionTime >= SIDE_ALIGN_MOVE_DURATION_MS) {
                        stopRobot();
                        currentAlignSideSubState = ALIGN_SIDE_MEASURE; // Re-measure after move
                        lastActionTime = currentMillis;
                    }
                    break;

                case ALIGN_SIDE_DONE:
                    currentRobotState = BEHAVIOR_COMPLETE;
                    Serial.println("Side wall alignment complete.");
                    oledShowText("Side Aligned!", "Dist: " + String(targetSideDistanceCM) + "cm");
                    break;
            }
            break;

        case ALIGNING_TO_LIGHT_SOURCE: // Light sensor alignment
            switch (currentAlignLightSubState) {
                case ALIGN_LIGHT_INIT:
                    currentAlignLightSubState = ALIGN_LIGHT_MEASURE;
                    lastActionTime = currentMillis;
                    break;

                case ALIGN_LIGHT_MEASURE:
                    if (currentMillis - lastActionTime >= 50) { // Delay before taking readings
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
                            // Keep confirming
                            currentAlignLightSubState = ALIGN_LIGHT_MEASURE;
                            lastActionTime = currentMillis;
                        }
                    } else if (currentLeftLight > currentRightLight) { // Left sensor brighter, move left
                        Serial.println("Left light brighter, moving LEFT");
                        moveRobot(moveLeft, STRAFE_SPEED);
                        currentAlignLightSubState = ALIGN_LIGHT_MOVE_DELAY;
                        lastActionTime = currentMillis;
                        stableLightCount = 0; // Reset confirmation
                    } else { // Right sensor brighter, move right
                        Serial.println("Right light brighter, moving RIGHT");
                        moveRobot(moveRight, STRAFE_SPEED);
                        currentAlignLightSubState = ALIGN_LIGHT_MOVE_DELAY;
                        lastActionTime = currentMillis;
                        stableLightCount = 0; // Reset confirmation
                    }
                    break;

                case ALIGN_LIGHT_MOVE_DELAY:
                    if (currentMillis - lastActionTime >= LIGHT_SENSOR_MOVE_DURATION_MS) {
                        stopRobot();
                        currentAlignLightSubState = ALIGN_LIGHT_MEASURE; // Re-measure after move
                        lastActionTime = currentMillis;
                    }
                    break;

                case ALIGN_LIGHT_DONE:
                    currentRobotState = BEHAVIOR_COMPLETE;
                    Serial.println("Light source alignment complete.");
                    oledShowText("Light Aligned!", "Center.");
                    break;
            }
            break;
                case ALIGNING_POST_TURN_FRONT_DISTANCE:
            switch (currentPostTurnFrontSubState) {
                case POST_TURN_FRONT_INIT:
                    stopRobot();
                    currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_LEFT;
                    lastActionTime = currentMillis;
                    break;

                case POST_TURN_FRONT_MEASURE_LEFT:
                    if (currentMillis - lastActionTime >= 60) {
                        currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
                        lastActionTime = currentMillis;
                        currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_RIGHT;
                    }
                    break;

                case POST_TURN_FRONT_MEASURE_RIGHT:
                    if (currentMillis - lastActionTime >= 60) {
                        currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
                        lastActionTime = currentMillis;
                        currentPostTurnFrontSubState = POST_TURN_FRONT_CHECK;
                    }
                    break;

                case POST_TURN_FRONT_CHECK: {
                    oledShowText("Post-Turn Front",
                                 "L:" + String(currentLeftDist) + " R:" + String(currentRightDist));

                    if (currentLeftDist >= 300 || currentRightDist >= 300 ||
                        currentLeftDist <= 0   || currentRightDist <= 0) {
                        stopRobot();
                        Serial.println("Invalid post-turn front reading, re-measuring...");
                        currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_LEFT;
                        lastActionTime = currentMillis;
                        break;
                    }

                    int averageDist = (currentLeftDist + currentRightDist) / 2;

                    if (abs(averageDist - targetPostTurnFrontDistanceCM) <= DISTANCE_TOLERANCE_CM) {
                        stopRobot();
                        Serial.println("Post-turn front distance achieved.");
                        currentPostTurnFrontSubState = POST_TURN_FRONT_DONE;
                    } else if (averageDist > targetPostTurnFrontDistanceCM) {
                        Serial.println("Too far from post-turn front wall, moving FORWARD");
                        moveRobot(moveForward, MAIN_SPEED);
                        currentPostTurnFrontSubState = POST_TURN_FRONT_MOVE_DELAY;
                        lastActionTime = currentMillis;
                    } else {
                        Serial.println("Too close to post-turn front wall, moving BACKWARD");
                        moveRobot(moveBackward, MAIN_SPEED);
                        currentPostTurnFrontSubState = POST_TURN_FRONT_MOVE_DELAY;
                        lastActionTime = currentMillis;
                    }
                    break;
                }

                case POST_TURN_FRONT_MOVE_DELAY:
                    if (currentMillis - lastActionTime >= POST_TURN_FRONT_MOVE_DURATION_MS) {
                        stopRobot();
                        currentPostTurnFrontSubState = POST_TURN_FRONT_MEASURE_LEFT;
                        lastActionTime = currentMillis;
                    }
                    break;

                case POST_TURN_FRONT_DONE:
                    currentRobotState = BEHAVIOR_COMPLETE;
                    oledShowText("Post-Turn Front", "Done");
                    break;
            }
            break;

        case DETECTING_COLOR: // Color detection
            switch (currentColorTurnSubState) {
                case COLOR_TURN_INIT:
                    stopRobot(); // Ensure robot is stopped for accurate reading
                    Serial.println("Starting color detection...");
                    currentColorTurnSubState = COLOR_TURN_DETECT;
                    lastActionTime = currentMillis;
                    break;

                case COLOR_TURN_DETECT:
                    if (currentMillis - lastActionTime >= 100) { // Small delay before reading color
                        detectedColor = detectFloorColor();
                        if (detectedColor != 0) { // If color detected
                            lastDetectedColor = detectedColor;   // remember result for mission flow
                            currentColorTurnSubState = COLOR_TURN_ACTION;
                            lastActionTime = currentMillis;
                        } else {
                            // If no clear color, try again after a delay
                            Serial.println("No clear color, re-detecting...");
                            lastActionTime = currentMillis;
                        }
                    }
                    break;

                case COLOR_TURN_ACTION:
                    resetEncoderCounts();
                    turnStartTicks = getAverageAbsoluteEncoderCount();
                    turnStartTime = currentMillis;

                    if (detectedColor == 1) { // Green -> turn LEFT
                        Serial.println("Green detected, turning LEFT (Counterclockwise)");
                        moveRobot(moveCounterclockwise, TURN_SPEED);
                    } else if (detectedColor == 2) { // Red -> turn RIGHT
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
                    long currentTicks = getAverageAbsoluteEncoderCount();

                    if (currentTicks - turnStartTicks >= TURN_90_DEGREE_ENCODER_TICKS) {
                        stopRobot();
                        delay(200);   // small settling delay
                        resetEncoderCounts();
                        Serial.println("Encoder-based 90 degree turn complete.");
                        currentColorTurnSubState = COLOR_TURN_DONE;
                    } 
                    else if (currentMillis - turnStartTime >= TURN_90_TIMEOUT_MS) {
                        stopRobot();
                        delay(200);   // small settling delay
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
            break;
            case ALIGNING_POST_TURN_ANGLE:
    switch (currentPostTurnAngleSubState) {
        case POST_TURN_ANGLE_INIT:
            stopRobot();
            currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_LEFT;
            lastActionTime = currentMillis;
            break;

        case POST_TURN_ANGLE_MEASURE_LEFT:
            if (currentMillis - lastActionTime >= 60) {
                currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
                lastActionTime = currentMillis;
                currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_RIGHT;
            }
            break;

        case POST_TURN_ANGLE_MEASURE_RIGHT:
            if (currentMillis - lastActionTime >= 60) {
                currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
                lastActionTime = currentMillis;
                currentPostTurnAngleSubState = POST_TURN_ANGLE_CHECK;
            }
            break;

        case POST_TURN_ANGLE_CHECK:
            oledShowText("Post-Turn Angle",
                         "L:" + String(currentLeftDist) + " R:" + String(currentRightDist));

            if (currentLeftDist >= 300 || currentRightDist >= 300 ||
                currentLeftDist <= 0   || currentRightDist <= 0) {
                stopRobot();
                Serial.println("Invalid post-turn angle reading, re-measuring...");
                currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_LEFT;
                lastActionTime = currentMillis;
                break;
            }

            if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
                stopRobot();
                Serial.println("Post-turn angle alignment complete.");
                currentPostTurnAngleSubState = POST_TURN_ANGLE_DONE;
            } else if (currentLeftDist > currentRightDist) {
                Serial.println("Left > Right, turning RIGHT");
                moveRobot(moveClockwise, TURN_SPEED);
                currentPostTurnAngleSubState = POST_TURN_ANGLE_TURN_DELAY;
                lastActionTime = currentMillis;
            } else {
                Serial.println("Right > Left, turning LEFT");
                moveRobot(moveCounterclockwise, TURN_SPEED);
                currentPostTurnAngleSubState = POST_TURN_ANGLE_TURN_DELAY;
                lastActionTime = currentMillis;
            }
            break;

        case POST_TURN_ANGLE_TURN_DELAY:
            if (currentMillis - lastActionTime >= ALIGN_TURN_DURATION_MS) {
                stopRobot();
                delay(80);
                resetEncoderCounts();
                currentPostTurnAngleSubState = POST_TURN_ANGLE_MEASURE_LEFT;
                lastActionTime = currentMillis;
            }
            break;

        case POST_TURN_ANGLE_DONE:
            currentRobotState = BEHAVIOR_COMPLETE;
            oledShowText("Post-Turn Angle", "Done");
            break;
    }
    break;

        case BEHAVIOR_COMPLETE:
			// A specific behavior (like move or align) has completed.
			// The main loop (in .ino) will decide the next action.
			break;
    }
}