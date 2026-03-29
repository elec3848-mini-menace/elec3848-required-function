/**
 * @file behavior.cpp
 * @brief Behavior dispatcher and orchestration wrappers.
 *
 * Individual behavior implementations are split across dedicated source files.
 * This file only owns shared lifecycle state and routes updates to the active
 * behavior handler.
 */
#include "behavior.h"
#include "behavior_internal.h"

RobotState currentRobotState = IDLE;

void setupBehaviors() {
    currentRobotState = IDLE;
    Serial.println("Behaviors setup complete.");
}

bool isBehaviorIdle() {
    return currentRobotState == IDLE;
}

bool tickBehaviorUntilComplete() {
    updateBehaviors();

    if (currentRobotState == BEHAVIOR_COMPLETE) {
        currentRobotState = IDLE;
        return true;
    }

    return false;
}

void updateBehaviors() {
    switch (currentRobotState) {
        case IDLE:
            // Robot is idle, waiting for a command.
            break;

        case MOVING_FORWARD_DISTANCE:
            updateMovingForwardDistance();
            break;

        case ALIGNING_WALL:
            updateAligningWall();
            break;

        case ALIGNING_SIDE_WALL_RIGHT:
            updateAligningSideWallRight();
            break;

        case ALIGNING_SIDE_WALL_LEFT:
            updateAligningSideWallLeft();
            break;

        case ALIGNING_TO_LIGHT_SOURCE:
            updateAligningToLightSource();
            break;

        case DETECTING_COLOR:
            updateDetectingColor();
            break;

        case ALIGNING_POST_TURN_FRONT_DISTANCE:
            updateAligningPostTurnFrontDistance();
            break;

        case ALIGNING_POST_TURN_ANGLE:
            updateAligningPostTurnAngle();
            break;

        case TURNING_BASED_ON_COLOR:
            // Reserved for older flow; no active behavior currently uses this.
            break;

        case BEHAVIOR_COMPLETE:
            // Completion is consumed by tickBehaviorUntilComplete().
            break;
    }
}
