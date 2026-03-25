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
    MISSION_STEP_MOVE,          // Move forward a certain distance
    MISSION_STEP_WAIT_0,
    MISSION_STEP_ALIGN_FRONT_1, // Align to front wall (first time)
    MISSION_STEP_WAIT_1,
    MISSION_STEP_ALIGN_SIDE,    // Align to right side wall
    MISSION_STEP_WAIT_2,
    MISSION_STEP_ALIGN_LIGHT,   // Align to light source
    MISSION_STEP_WAIT_3,
    MISSION_STEP_ALIGN_FRONT_2, // Re-align to front wall
    MISSION_STEP_WAIT_4,
    MISSION_STEP_DETECT_COLOR,  // Detect color and turn
    MISSION_STEP_WAIT_5,
    MISSION_STEP_ALIGN_FRONT_3, // Final align to front wall
    MISSION_STEP_WAIT_6,
    MISSION_DONE_FINAL
};
MissionState currentMissionState = MISSION_INIT;
unsigned long lastMissionActionTime = 0;
const unsigned long SHORT_ACTION_DELAY = 300;  // Short delay for display updates or brief pauses

void setup() {
    Serial.begin(115200);
    Serial.println("Robot Starting Up...");

    setupMotor();
    setupDisplay();
    setupSensors(); // This now sets up all sensors (US, Light, Color)
		//calibrateLightSensor(); // calibrate -- turns out don't really need 

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
				Serial.println("Mission: Starting Step 0 (Move forward)");
				startMoveForwardDistanceCM(MOVE_FORWARD_DISTANCE_CM);
				currentMissionState = MISSION_STEP_MOVE;
			}
			break;

    case MISSION_STEP_MOVE:
			// Wait for the move forward behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 0 Complete.");
				oledShowText("Moved", "Next: Align");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_WAIT_0;
				currentRobotState = IDLE;  // Reset behavior state for the next command
			}
			break;

    case MISSION_STEP_WAIT_0:
			// Short delay
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: Starting Step 1 (Align with Wall)");
				startAlignWithWallUsingUltrasonic();
				currentMissionState = MISSION_STEP_ALIGN_FRONT_1;
			}
			break;

		case MISSION_STEP_ALIGN_FRONT_1:
			// Wait for the front wall alignment behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 1 (Front Align) Complete.");
				oledShowText("Front Aligned!", "Next: Side Align");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_WAIT_1;
				currentRobotState = IDLE;  // Reset behavior state for the next command
			}
			break;

		case MISSION_STEP_WAIT_1:
			// Short delay
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: Starting Step 2 (Align with Side Wall)");
				startAlignWithSideWall(TARGET_SIDE_WALL_DISTANCE_CM);
				currentMissionState = MISSION_STEP_ALIGN_SIDE;
			}
			break;

		case MISSION_STEP_ALIGN_SIDE:
			// Wait for the side wall alignment behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 2 (Side Align) Complete.");
				oledShowText("Side Aligned!", "Next: Light Align");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_WAIT_2;
				currentRobotState = IDLE;
			}
			break;

		case MISSION_STEP_WAIT_2:
			// Short delay
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: Starting Step 3 (Align to Light Source)");
				startAlignToLightSource();
				currentMissionState = MISSION_STEP_ALIGN_LIGHT;
			}
			break;

		case MISSION_STEP_ALIGN_LIGHT:
			// Wait for the light source alignment behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 3 (Light Align) Complete.");
				oledShowText("Light Aligned!", "Next: Front Align");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_WAIT_3;
				currentRobotState = IDLE;
			}
			break;

		case MISSION_STEP_WAIT_3:
			// Short delay
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: Starting Step 4 (Re-align with Front Wall)");
				startAlignWithWallUsingUltrasonic(); // Re-use front wall alignment
				currentMissionState = MISSION_STEP_ALIGN_FRONT_2;
			}
			break;

		case MISSION_STEP_ALIGN_FRONT_2:
			// Wait for the front wall alignment behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 4 (Front Align 2) Complete.");
				oledShowText("Front Aligned!", "Next: Color Detect");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_WAIT_4;
				currentRobotState = IDLE;
			}
			break;

		case MISSION_STEP_WAIT_4:
			// Short delay
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: Starting Step 5 (Detect Color and Turn)");
				startDetectColorAndTurn();
				currentMissionState = MISSION_STEP_DETECT_COLOR;
			}
			break;

		case MISSION_STEP_DETECT_COLOR:
			// Wait for color detection and turn behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 5 (Color Detect & Turn) Complete.");
				oledShowText("Turned!", "Next: Final Align");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_WAIT_5;
				currentRobotState = IDLE;
			}
			break;

		case MISSION_STEP_WAIT_5:
			// Short delay
			if (currentMillis - lastMissionActionTime >= SHORT_ACTION_DELAY) {
				Serial.println("Mission: Starting Step 6 (Final Align with Front Wall)");
				startAlignWithWallUsingUltrasonic(); // Re-use front wall alignment
				currentMissionState = MISSION_STEP_ALIGN_FRONT_3;
			}
			break;

		case MISSION_STEP_ALIGN_FRONT_3:
			// Wait for the final front wall alignment behavior to complete
			if (currentRobotState == BEHAVIOR_COMPLETE) {
				Serial.println("Mission: Step 6 (Front Align 3) Complete.");
				oledShowText("Final Align!", "Mission Done");
				lastMissionActionTime = currentMillis;
				currentMissionState = MISSION_STEP_WAIT_6;
				stopRobot();               // Ensure robot is stopped after alignment
				currentRobotState = IDLE;  // Reset behavior state
			}
			break;

		case MISSION_STEP_WAIT_6:
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