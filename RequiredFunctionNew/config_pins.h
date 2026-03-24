#pragma once

// OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define OLED_RESET 28
#define SCREEN_ADDRESS 0x3C

// Movement tuning
#define MAIN_SPEED 60
#define TURN_SPEED 110
#define MOTOR_SYNC_DELAY 250

const float TICKS_PER_CM = 18.0;   // tune: Number of encoder ticks for 1 cm of wheel travel
const float CM_PER_TICK = 1.0 / TICKS_PER_CM; // Distance in CM per single encoder tick (for localization)
const float ROBOT_WHEEL_BASE_CM = 20.0; // **IMPORTANT: Tune this value based on your robot's physical dimensions (distance between left and right wheels)**
const int WALL_EQUAL_TOLERANCE_CM = 4;

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

#define BACK_LEFT_US_TRIG A9
#define BACK_LEFT_US_ECHO A8
#define BACK_RIGHT_US_TRIG A11
#define BACK_RIGHT_US_ECHO A10

// Movement IDs
#define moveForward 0
#define moveBackward 1
#define moveLeft 2
#define moveRight 3
#define moveClockwise 4
#define moveCounterclockwise 5