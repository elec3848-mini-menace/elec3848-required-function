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
    oledShowText("Aligning Front", "L:--- R:---");
}

// Function to start aligning with the side wall
void startAlignWithSideWall(float distanceCM) {
    Serial.print("Start side wall alignment to ");
    Serial.print(distanceCM);
    Serial.println("cm");
    currentRobotState = ALIGNING_SIDE_WALL;
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
    oledShowText("Detecting Color", "...");
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
					if (currentMillis - lastActionTime >= 50) { // Delay before taking left ultrasonic reading
						currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
						lastActionTime = currentMillis; // Reset timer for next action
						currentAlignSubState = ALIGN_MEASURE_RIGHT;
					}
					break;

				case ALIGN_MEASURE_RIGHT:
					if (currentMillis - lastActionTime >= 50) { // Delay before taking right ultrasonic reading
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
						Serial.println("Invalid front ultrasonic reading, re-measuring...");
						currentAlignSubState = ALIGN_MEASURE_LEFT; // Go back to measure
						lastActionTime = currentMillis; // Delay before next measurement
						break;
					}

					if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
						// Initial alignment detected, start confirmation process
						stopRobot(); // Stop while confirming alignment
						Serial.println("Front alignment detected, starting confirmation...");
						alignmentConfirmationCount = 0;
						currentAlignSubState = ALIGN_CONFIRM_MEASURE_LEFT;
						lastActionTime = currentMillis; // Reset for measurement delay in confirming state
					} else if (currentLeftDist > currentRightDist) {
						Serial.println("Turn RIGHT");
						moveRobot(moveClockwise, TURN_SPEED);
						currentAlignSubState = ALIGN_TURN_DELAY;
						lastActionTime = currentMillis; // Start timer for turn duration
					} else { // rightDist > leftDist
						Serial.println("Turn LEFT");
						moveRobot(moveCounterclockwise, TURN_SPEED);
						currentAlignSubState = ALIGN_TURN_DELAY;
						lastActionTime = currentMillis; // Start timer for turn duration
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
					// wait interval to let reflections settle before pinging left
					if (currentMillis - lastActionTime >= 10) {
						currentLeftDist = getUltrasonicCM(US_FRONT_LEFT);
						lastActionTime = currentMillis;

						// next: schedule right measurement after a gap
						currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST_MEASURE_RIGHT;
					}
					break;

				case ALIGN_MOVING_TO_TARGET_DIST_MEASURE_RIGHT:
					if (currentMillis - lastActionTime >= 10) {
						currentRightDist = getUltrasonicCM(US_FRONT_RIGHT);
						lastActionTime = currentMillis;

						// Check for invalid ultrasonic readings
						if (currentLeftDist >= 300 || currentRightDist >= 300 ||
							currentLeftDist <= 0   || currentRightDist <= 0) {
							stopRobot();
							Serial.println("Invalid ultrasonic reading while moving to target, stopping.");
							currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST; // keep trying to move to target distance
							break;
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
							// keep moving: next cycle starts with left again
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

        case ALIGNING_SIDE_WALL:
            switch (currentAlignSideSubState) {
                case ALIGN_SIDE_INIT:
                    currentAlignSideSubState = ALIGN_SIDE_MEASURE;
                    lastActionTime = currentMillis;
                    break;

                case ALIGN_SIDE_MEASURE:
                    if (currentMillis - lastActionTime >= 50) { // Delay before taking reading
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
                    if (detectedColor == 1) { // Green detected -> turn LEFT (Counterclockwise)
                        Serial.println("Green detected, turning LEFT (Counterclockwise)");
                        moveRobot(moveCounterclockwise, TURN_SPEED);
                    } else if (detectedColor == 2) { // Red detected -> turn RIGHT (Clockwise)
                        Serial.println("Red detected, turning RIGHT (Clockwise)");
                        moveRobot(moveClockwise, TURN_SPEED);
                    } else {
                        // Should not happen if detectedColor is checked, but as a fallback
                        Serial.println("Unknown color, stopping.");
                        stopRobot();
                        currentColorTurnSubState = COLOR_TURN_DONE;
                        break;
                    }
                    oledShowText("Color: " + String(detectedColor == 1 ? "Green" : "Red"), "Turning...");
                    currentColorTurnSubState = COLOR_TURN_DELAY;
                    lastActionTime = currentMillis;
                    break;

                case COLOR_TURN_DELAY:
                    if (currentMillis - lastActionTime >= TURN_90_DEGREE_DURATION_MS) {
                        stopRobot();
                        Serial.println("Turn complete.");
                        currentColorTurnSubState = COLOR_TURN_DONE;
                    }
                    break;

                case COLOR_TURN_DONE:
                    currentRobotState = BEHAVIOR_COMPLETE;
                    Serial.println("Color detection and turn behavior complete.");
                    oledShowText("Turn Complete!", "Next Step...");
                    break;
            }
            break;

        case BEHAVIOR_COMPLETE:
			// A specific behavior (like move or align) has completed.
			// The main loop (in .ino) will decide the next action.
			break;
    }
}