#pragma once
#include <Arduino.h>

// Initialize motor GPIO + attach encoder interrupts
void setupMotor();

// Move robot in a given direction with PWM
void moveRobot(int dir, int pwm);

// Stop all motors
void stopRobot();

// Reset encoder counts
void resetEncoderCounts();

// Average encoder ticks (for distance moves)
long getAverageEncoderCount();

// Perform encoder-based motor sync adjustments (call periodically)
void motorSyncService();