/**
 * @file main.ino
 * @brief Top-level Arduino sketch for the ELEC3848 mecanum-wheel robot.
 *
 * Contains a table-driven mission orchestrator. Each mission step has a
 * dedicated begin function, while behavior wrappers advance and report
 * completion of the active non-blocking behavior.
 */
#include <Arduino.h>
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"
#include "behavior.h"
#include "oled.h"

// Variables for motor synchronization
unsigned long lastMotorSyncTime = 0;
const unsigned long MOTOR_SYNC_INTERVAL = 1;
const unsigned long SHORT_ACTION_DELAY = 300;

struct MissionStep {
    const char* name;
    void (*begin)();
};

size_t currentStepIndex = 0;
bool waitingBetweenSteps = false;
unsigned long stepTransitionTime = 0;
bool missionFinished = false;

static void beginStepMoveForward() {
    Serial.println("Mission: Step 0 - Move forward");
    startMoveForwardDistanceCM(MOVE_FORWARD_DISTANCE_CM);
}

static void beginStepAlignFront1() {
    Serial.println("Mission: Step 1 - Front wall align");
    startAlignWithWallUsingUltrasonic();
}

static void beginStepAlignSideRight() {
    Serial.println("Mission: Step 2 - Side wall align (right sensor)");
    startAlignWithSideWallRight(TARGET_SIDE_WALL_DISTANCE_CM);
}

static void beginStepAlignLight() {
    Serial.println("Mission: Step 3 - Light source align");
    startAlignToLightSource();
}

static void beginStepAlignFront2() {
    Serial.println("Mission: Step 4 - Front wall re-align");
    startAlignWithWallUsingUltrasonic();
}

static void beginStepDetectColorTurn() {
    Serial.println("Mission: Step 5 - Detect color and turn");
    startDetectColorAndTurn();
}

static void beginStepPostTurnAngle() {
    Serial.println("Mission: Step 6 - Post-turn angle align");
    startAlignPostTurnAngle();
}

static void beginStepPostTurnFront() {
    Serial.println("Mission: Step 7 - Post-turn front distance");
    startAlignPostTurnFrontDistance(POST_TURN_FRONT_TARGET_CM);
}

static void beginStepPostTurnSide() {
    const int lastColor = getLastDetectedColor();

    if (lastColor == 1) {
        // Green -> turn was left -> side wall is on the right sensor.
        Serial.println("Mission: Step 8 - Post-turn side align (right sensor)");
        startAlignWithSideWallRight(POST_TURN_SIDE_TARGET_CM);
    } else if (lastColor == 2) {
        // Red -> turn was right -> side wall is on the left sensor.
        Serial.println("Mission: Step 8 - Post-turn side align (left sensor)");
        startAlignWithSideWallLeft(POST_TURN_SIDE_TARGET_CM);
    } else {
        Serial.println("Mission: Step 8 - Unknown color, default right sensor");
        startAlignWithSideWallRight(POST_TURN_SIDE_TARGET_CM);
    }
}

static const MissionStep kMissionSteps[] = {
    {"Move Forward", beginStepMoveForward},
    {"Align Front", beginStepAlignFront1},
    {"Align Side", beginStepAlignSideRight},
    {"Align Light", beginStepAlignLight},
    {"Re-Align Front", beginStepAlignFront2},
    {"Detect Color Turn", beginStepDetectColorTurn},
    {"Post-Turn Angle", beginStepPostTurnAngle},
    {"Post-Turn Front", beginStepPostTurnFront},
    {"Post-Turn Side", beginStepPostTurnSide},
};

static const size_t kMissionStepCount = sizeof(kMissionSteps) / sizeof(kMissionSteps[0]);

static void beginCurrentMissionStep() {
    if (currentStepIndex >= kMissionStepCount) {
        missionFinished = true;
        stopRobot();
        oledShowText("Mission Complete", "Robot Idle");
        Serial.println("Mission: All steps complete.");
        return;
    }

    Serial.print("Mission: Starting ");
    Serial.print(currentStepIndex);
    Serial.print(" - ");
    Serial.println(kMissionSteps[currentStepIndex].name);
    kMissionSteps[currentStepIndex].begin();
}

static void updateMissionRunner(unsigned long currentMillis) {
    if (missionFinished) {
        return;
    }

    if (waitingBetweenSteps) {
        if (currentMillis - stepTransitionTime >= SHORT_ACTION_DELAY) {
            waitingBetweenSteps = false;
            beginCurrentMissionStep();
        }
        return;
    }

    if (tickBehaviorUntilComplete()) {
        Serial.print("Mission: Step complete -> ");
        Serial.println(kMissionSteps[currentStepIndex].name);
        currentStepIndex++;
        stepTransitionTime = currentMillis;
        waitingBetweenSteps = true;
    }
}

void setup() {
	Serial.begin(115200);
	Serial.println("Robot Starting Up...");

	setupMotor();
	setupDisplay();
	setupSensors();

	setupBehaviors();

	oledShowText("Robot Ready", "Starting...");
	stopRobot();
	delay(1000);

	beginCurrentMissionStep();
}

void loop() {
	const unsigned long currentMillis = millis();

	if (currentMillis - lastMotorSyncTime >= MOTOR_SYNC_INTERVAL) {
		motorSyncService();
		lastMotorSyncTime = currentMillis;
	}

	updateMissionRunner(currentMillis);
} // end loop()
