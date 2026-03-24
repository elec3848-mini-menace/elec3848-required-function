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
    ALIGN_COMPLETE
};
static AlignWallSubState currentAlignSubState = ALIGN_INIT;
static unsigned long lastActionTime = 0;
static int currentLeftDist = 0;
static int currentRightDist = 0;

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
    lastActionTime = millis(); // Initialize timer
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

					if (currentLeftDist >= 300 || currentRightDist >= 300 || currentLeftDist <= 0) {
						stopRobot();
						Serial.println("Invalid ultrasonic reading, re-measuring...");
						currentAlignSubState = ALIGN_MEASURE_LEFT; // Go back to measure
						lastActionTime = currentMillis; // Delay before next measurement
						break;
					}

					if (abs(currentLeftDist - currentRightDist) <= WALL_EQUAL_TOLERANCE_CM) {
						stopRobot();
						Serial.println("Wall aligned within tolerance.");
						currentAlignSubState = ALIGN_COMPLETE;
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

				case ALIGN_COMPLETE:
						currentRobotState = BEHAVIOR_COMPLETE;
						Serial.println("Wall alignment complete.");
						oledShowText("Wall Aligned!", "Done.");
					break;
			}
			break;

			case BEHAVIOR_COMPLETE:
			// A specific behavior (like move or align) has completed.
			// The main loop (in .ino) will decide the next action.
			break;
    }
}