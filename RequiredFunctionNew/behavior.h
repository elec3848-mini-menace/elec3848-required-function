#pragma once
#include <Arduino.h>

// Enum for overall robot states
enum RobotState {
    IDLE,
    MOVING_FORWARD_DISTANCE,
    ALIGNING_WALL,
    ALIGNING_SIDE_WALL_RIGHT,
    ALIGNING_SIDE_WALL_LEFT,
    ALIGNING_TO_LIGHT_SOURCE,
    DETECTING_COLOR,
    ALIGNING_POST_TURN_FRONT_DISTANCE,
    TURNING_BASED_ON_COLOR,
    BEHAVIOR_COMPLETE
};

extern RobotState currentRobotState;

// Initialize behavior state machine
void setupBehaviors();

// Non-blocking update function for current behavior
void updateBehaviors();

// Initiates a non-blocking move
void startMoveForwardDistanceCM(float distanceCM);

// Initiates a non-blocking wall alignment
void startAlignWithWallUsingUltrasonic();

// Function to start aligning with the side wall
void startAlignWithSideWallRight(float targetDistanceCM);
void startAlignWithSideWallLeft(float targetDistanceCM);

// Function to start aligning to a light source
void startAlignToLightSource();

// Function to start detecting color and turning
void startDetectColorAndTurn();

// NEW: post-turn front distance adjustment
void startAlignPostTurnFrontDistance(float targetDistanceCM);

// NEW: allow .ino to know which way robot turned
int getLastDetectedColor();