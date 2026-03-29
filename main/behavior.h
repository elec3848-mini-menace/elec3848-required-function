/**
 * @file behavior.h
 * @brief Non-blocking robot behavior state machine interface.
 *
 * Each "start" function initiates a behavior and returns immediately.
 * For mission orchestration, prefer tickBehaviorUntilComplete() instead of
 * directly reading or writing currentRobotState.
 *
 * Note for maintainers:
 * This is the legacy behavior surface that exposes a shared RobotState.
 * main.ino now uses a table-driven mission runner on top of this API.
 * A future cleanup can replace this shared enum with per-behavior begin/update
 * pairs that directly return completion status.
 */
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
    ALIGNING_POST_TURN_ANGLE,
    TURNING_BASED_ON_COLOR,
    BEHAVIOR_COMPLETE
};

extern RobotState currentRobotState;

// Initialise the behavior state machine; must be called once in setup().
void setupBehaviors();

// Advance the active behavior by one step. Call every iteration of loop().
void updateBehaviors();

// Wrapper API for orchestrators:
// Tick active behavior and return true once when it completes.
// This consumes completion by resetting internal state back to IDLE.
bool tickBehaviorUntilComplete();

// Return true when no behavior is currently active.
bool isBehaviorIdle();

// Begin a non-blocking encoder-counted forward move of the given distance.
void startMoveForwardDistanceCM(float distanceCM);

// Begin non-blocking angular alignment with the front wall using the two
// front ultrasonic sensors, then drive to TARGET_WALL_DISTANCE_CM.
void startAlignWithWallUsingUltrasonic();

// Begin non-blocking lateral alignment to targetDistanceCM from the right
// side wall using the right-facing ultrasonic sensor.
void startAlignWithSideWallRight(float targetDistanceCM);

// Begin non-blocking lateral alignment to targetDistanceCM from the left
// side wall using the left-facing ultrasonic sensor.
void startAlignWithSideWallLeft(float targetDistanceCM);

// Begin non-blocking lateral alignment centred on the overhead light source
// using the two analogue light sensors.
void startAlignToLightSource();

// Begin non-blocking floor colour detection followed by a 90-degree turn
// (counter-clockwise for green, clockwise for red).
void startDetectColorAndTurn();

// Begin non-blocking angular correction after a 90-degree turn using the
// front ultrasonic sensors.
void startAlignPostTurnAngle();

// Begin non-blocking forward/backward adjustment to reach targetDistanceCM
// from the front wall after a turn.
void startAlignPostTurnFrontDistance(float targetDistanceCM);

// Return the colour detected by the most recent startDetectColorAndTurn call.
// 1 = green, 2 = red, 0 = unknown.
int getLastDetectedColor();