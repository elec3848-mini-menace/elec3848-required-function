#pragma once
#include <Arduino.h>

// Enum for overall robot states
enum RobotState {
    IDLE,
    MOVING_FORWARD_DISTANCE,
    ALIGNING_WALL,
    BEHAVIOR_COMPLETE // Indicates a specific behavior task has finished
};

extern RobotState currentRobotState; // Global state variable

void setupBehaviors(); // Initialize behavior state machine
void updateBehaviors(); // Non-blocking update function for current behavior
void startMoveForwardDistanceCM(float distanceCM); // Initiates a non-blocking move
void startAlignWithWallUsingUltrasonic(); // Initiates a non-blocking wall alignment