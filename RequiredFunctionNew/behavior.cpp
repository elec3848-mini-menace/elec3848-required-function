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

// Variables for alignWithWallUsingUltrasonic state machine
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
    Serial.println("Start wall alignment");
    currentRobotState = ALIGNING_WALL;
    currentAlignSubState = ALIGN_INIT;
    lastActionTime = millis();
    alignmentConfirmationCount = 0;
    oledShowText("Aligning Wall", "L:--- R:---");
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

        case ALIGNING_WALL:
			switch (currentAlignSubState) {
				case ALIGN_INIT:
					// Transition to first measurement
					currentAlignSubState = ALIGN_MEASURE_LEFT;
					lastActionTime = currentMillis; // Start timer for measurement
					break;

				case ALIGN_MEASURE_LEFT:
					if (currentMillis - lastActionTime >= 50) { // Delay before taking left ultrasonic reading
						currentLeftDist = getUltrasonicCM(0);
						lastActionTime = currentMillis; // Reset timer for next action
						currentAlignSubState = ALIGN_MEASURE_RIGHT;
					}
					break;

				case ALIGN_MEASURE_RIGHT:
					if (currentMillis - lastActionTime >= 50) { // Delay before taking right ultrasonic reading
						currentRightDist = getUltrasonicCM(1);
						lastActionTime = currentMillis; // Reset timer for next action
						currentAlignSubState = ALIGN_CHECK;
					}
					break;

				case ALIGN_CHECK:
					oledShowText("Aligning Wall", "L:" + String(currentLeftDist) + " R:" + String(currentRightDist));

					// Check for invalid ultrasonic readings (out of range or 0)
					if (currentLeftDist >= 300 || currentRightDist >= 300 || currentLeftDist <= 0 || currentRightDist <= 0) {
						stopRobot();
						Serial.println("Invalid ultrasonic reading, re-measuring...");
						currentAlignSubState = ALIGN_MEASURE_LEFT; // Go back to measure
						lastActionTime = currentMillis; // Delay before next measurement
						break;
					}

					if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
						// Initial alignment detected, start confirmation process
						stopRobot(); // Stop while confirming alignment
						Serial.println("Alignment detected, starting confirmation...");
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
					if (currentMillis - lastActionTime >= 120) { // Original turn delay was 120ms
						stopRobot();
						currentAlignSubState = ALIGN_MEASURE_LEFT; // Go back to measure after turn
						lastActionTime = currentMillis; // Delay before next measurement
					}
					break;

				case ALIGN_CONFIRM_MEASURE_LEFT:
					oledShowText("Confirming Align", "L:--- R:" + String(currentRightDist));

					if (currentMillis - lastActionTime >= 50) {   // wait before left measurement
						currentLeftDist = getUltrasonicCM(0);
						lastActionTime = currentMillis;             // start timer for right measurement
						currentAlignSubState = ALIGN_CONFIRM_MEASURE_RIGHT;
					}
					break;

				case ALIGN_CONFIRM_MEASURE_RIGHT:
					oledShowText("Confirming Align", "L:" + String(currentLeftDist) + " R:---");

					if (currentMillis - lastActionTime >= 50) {   // wait before right measurement
						currentRightDist = getUltrasonicCM(1);
						lastActionTime = currentMillis;             // timer for next confirmation cycle

						if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
							alignmentConfirmationCount++;
							Serial.print("Confirmed count: "); Serial.println(alignmentConfirmationCount);

							if (alignmentConfirmationCount >= ALIGNMENT_CONFIRMATION_THRESHOLD) {
								stopRobot();
								Serial.println("Wall alignment confirmed and stable.");
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
                    Serial.println("cm");

                    // Kick off first left measurement
                    currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST_MEASURE_LEFT;
                    lastActionTime = currentMillis; // start timer for left
					break;

				case ALIGN_MOVING_TO_TARGET_DIST_MEASURE_LEFT:
					// wait interval to let reflections settle before pinging left
					if (currentMillis - lastActionTime >= 10) {
						currentLeftDist = getUltrasonicCM(0);
						lastActionTime = currentMillis;

						// next: schedule right measurement after a gap
						currentAlignSubState = ALIGN_MOVING_TO_TARGET_DIST_MEASURE_RIGHT;
					}
					break;

				case ALIGN_MOVING_TO_TARGET_DIST_MEASURE_RIGHT:
					if (currentMillis - lastActionTime >= 10) {
						currentRightDist = getUltrasonicCM(1);
						lastActionTime = currentMillis;

						// Check for invalid ultrasonic readings
						if (currentLeftDist >= 300 || currentRightDist >= 300 ||
							currentLeftDist <= 0   || currentRightDist <= 0) {
							stopRobot();
							Serial.println("Invalid ultrasonic reading while moving to target, stopping.");
							currentAlignSubState = ALIGN_DONE;
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

				case ALIGN_DONE: // NEW STATE: Final completion for the alignment behavior
					currentRobotState = BEHAVIOR_COMPLETE;
					Serial.println("Wall alignment and distance adjustment complete.");
					oledShowText("Wall Aligned!", "Dist: " + String(TARGET_WALL_DISTANCE_CM) + "cm");
					break;
			}
			break;

        case BEHAVIOR_COMPLETE:
			// A specific behavior (like move or align) has completed.
			// The main loop (in .ino) will decide the next action.
			break;
    }
}