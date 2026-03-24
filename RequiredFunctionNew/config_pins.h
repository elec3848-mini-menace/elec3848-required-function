// RequiredFunctionNew/config_pins.h
#pragma once

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET -1
#define SCREEN_ADDRESS 0x3C

// Movement tuning
const int MAIN_SPEED = 40;
const int TURN_SPEED = 40;
const int MOTOR_SYNC_DELAY = 250; // Delay before motor sync starts adjusting PWM

const float TICKS_PER_CM = 18.0;   // tune: Number of encoder ticks for 1 cm of wheel travel
const float CM_PER_TICK = 1.0 / TICKS_PER_CM;
const int WALL_EQUAL_TOLERANCE_CM = 1;

// Constants for enhanced wall alignment
const int ALIGNMENT_CONFIRMATION_THRESHOLD = 5;
const int TARGET_WALL_DISTANCE_CM = 8;
const int DISTANCE_TOLERANCE_CM = 1;

// Pins
#define FRONT_RIGHT_PWM 12
#define FRONT_RIGHT_BACKWARD 34
#define FRONT_RIGHT_FORWARD 35
#define FRONT_RIGHT_EN_A 18
#define FRONT_RIGHT_EN_B 31

#define FRONT_LEFT_PWM 8
#define FRONT_LEFT_BACKWARD 36
#define FRONT_LEFT_FORWARD 37
#define FRONT_LEFT_EN_A 19
#define FRONT_LEFT_EN_B 38

#define BACK_RIGHT_PWM 9
#define BACK_RIGHT_BACKWARD 43
#define BACK_RIGHT_FORWARD 42
#define BACK_RIGHT_EN_A 3
#define BACK_RIGHT_EN_B 49

#define BACK_LEFT_PWM 5
#define BACK_LEFT_BACKWARD A5
#define BACK_LEFT_FORWARD A4
#define BACK_LEFT_EN_A 2
#define BACK_LEFT_EN_B A1

#define LEFT_US_TRIG A9
#define LEFT_US_ECHO A8
#define RIGHT_US_TRIG A11
#define RIGHT_US_ECHO A10

// Movement IDs
#define moveForward 0
#define moveBackward 1
#define moveLeft 2
#define moveRight 3
#define moveClockwise 4
#define moveCounterclockwise 5