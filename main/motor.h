/**
 * @file motor.h
 * @brief Mecanum-wheel motor driver and quadrature encoder interface.
 *
 * Provides direction-based movement using six movement modes (forward,
 * backward, left, right, clockwise, counter-clockwise) and a background
 * motor-sync service that equalises wheel speeds using encoder feedback.
 */
#pragma once
#include <Arduino.h>

// Initialize motor GPIO and attach quadrature encoder interrupts.
void setupMotor();

// Set all four wheels to the given direction and PWM duty cycle (0-255).
// Direction constants (moveForward, moveLeft, etc.) are defined in config_pins.h.
void moveRobot(int dir, int pwm);

// Immediately cut power to all four wheels.
void stopRobot();

// Reset all four encoder tick counters to zero (interrupt-safe).
void resetEncoderCounts();

// Return the mean encoder count across all four wheels (signed).
long getAverageEncoderCount();

// Return the mean of the absolute encoder counts across all four wheels.
long getAverageAbsoluteEncoderCount();

// Individual raw encoder counts (useful for debugging and display).
long getFrontLeftEncoderCount();
long getFrontRightEncoderCount();
long getBackLeftEncoderCount();
long getBackRightEncoderCount();

// Adjust per-wheel PWM values to equalise wheel speeds. Call every 1 ms.
void motorSyncService();