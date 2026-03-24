#include <Arduino.h>

#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "behavior.h"
#include "oled.h"

// Variables for motor synchronization
unsigned long lastMotorSyncTime = 0;
const unsigned long MOTOR_SYNC_INTERVAL = 1;  // Call motorSyncService every 1ms

// Enum for the overall mission sequence
enum MissionState {
    MISSION_INIT,
    MISSION_STEP_1_MOVE,
    MISSION_STEP_1_WAIT,  // Wait after a behavior completes to display info
    MISSION_STEP_2_ALIGN,
    MISSION_STEP_2_WAIT,
    MISSION_DONE_FINAL
};
MissionState currentMissionState = MISSION_INIT;
unsigned long lastMissionActionTime = 0;
const unsigned long SHORT_ACTION_DELAY = 300;  // Short delay for display updates or brief pauses

void setup() {
    Serial.begin(115200);
    Serial.println("Robot Starting Up...");

    setupMotor();
    setupSensors();
    setupDisplay();
    setupBehaviors();  // Initialize the behavior state machine

    oledShowText("Robot Ready", "Waiting...");
    stopRobot();  // Ensure robot is stopped initially
    delay(1000);  // Initial delay to show "Ready" message
}

void loop() {
    unsigned long currentMillis = millis();

    // --- Motor Synchronization Service (called frequently) ---
    if (currentMillis - lastMotorSyncTime >= MOTOR_SYNC_INTERVAL) {
    	motorSyncService();
      	lastMotorSyncTime = currentMillis;
    }

    // --- Update Robot Behaviors ---
    updateBehaviors();

    // --- Mission Orchestrator (main state machine for mission flow) ---
    switch (currentMissionState) {
		case MISSION_INIT:
			// Wait until the robot's behavior state machine is idle before starting the mission
			if (currentRobotState == IDLE) {
				Serial.println("Mission: Starting Step 1 (Move Forward 30cm)");
				startMoveForwardDistanceCM(30.0);
				currentMissionState = MISSION_STEP_1_MOVE;
			}
			break;

		case MISSION_STEP_1_MOVE:
			// Wait for the move forward behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 1 Complete.");
				oledShowText("Moved 30cm", "Next: Align");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_1_WAIT;
				currentRobotState = IDLE;  // Reset behavior state for the next command
			}
			break;

		case MISSION_STEP_1_WAIT:
			// Short delay
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: Starting Step 2 (Align with Wall)");
				startAlignWithWallUsingUltrasonic();
				currentMissionState = MISSION_STEP_2_ALIGN;
			}
			break;

		case MISSION_STEP_2_ALIGN:
			// Wait for the align with wall behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 2 Complete.");
				oledShowText("Wall Aligned!", "Mission Done");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_2_WAIT;
				stopRobot();               // Ensure robot is stopped after alignment
				currentRobotState = IDLE;  // Reset behavior state
			}
			break;

		case MISSION_STEP_2_WAIT:
			// Short delay after mission completion
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: All steps complete.");
				oledShowText("Mission Complete", "Robot Idle");
				currentMissionState = MISSION_DONE_FINAL;
			}
			break;

		case MISSION_DONE_FINAL:
			// Mission is fully complete, robot is idle.
			// Can add logic here to wait for new commands or enter low-power mode.
			break;
    }
}