/**
 * @file behavior_move.cpp
 * @brief Forward distance behavior (encoder-counted).
 */
#include "behavior.h"
#include "behavior_internal.h"
#include "config_pins.h"
#include "motor.h"
#include "oled.h"

static float targetDistanceCM = 0;
static long targetEncoderTicks = 0;

void startMoveForwardDistanceCM(float distanceCM) {
    targetDistanceCM = distanceCM;
    targetEncoderTicks = static_cast<long>(distanceCM * TICKS_PER_CM);

    resetEncoderCounts();
    moveRobot(moveForward, MAIN_SPEED);
    currentRobotState = MOVING_FORWARD_DISTANCE;

    Serial.print("Starting move forward: ");
    Serial.print(distanceCM);
    Serial.println(" cm");
    oledShowText("Moving " + String(distanceCM) + "cm", "Enc: 0/" + String(targetEncoderTicks));
}

void updateMovingForwardDistance() {
    oledShowText("Moving " + String(targetDistanceCM) + "cm",
                 "Enc: " + String(getAverageEncoderCount()) + "/" + String(targetEncoderTicks));

    if (getAverageEncoderCount() >= targetEncoderTicks) {
        stopRobot();
        currentRobotState = BEHAVIOR_COMPLETE;
        Serial.println("Move forward complete.");
        oledShowText("Moved " + String(targetDistanceCM) + "cm", "Done.");
    }
}
