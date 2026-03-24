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

const float TICKS_PER_CM = 18.0;   // tune
const int WALL_EQUAL_TOLERANCE_CM = 4;

// Pins
#define FRONT_LEFT_PWM 12
#define FRONT_LEFT_BACKWARD 34
#define FRONT_LEFT_FORWARD 35
#define FRONT_LEFT_EN_A 18

#define FRONT_RIGHT_PWM 8
#define FRONT_RIGHT_BACKWARD 37
#define FRONT_RIGHT_FORWARD 36
#define FRONT_RIGHT_EN_A 19

#define BACK_LEFT_PWM 9
#define BACK_LEFT_BACKWARD 43
#define BACK_LEFT_FORWARD 42
#define BACK_LEFT_EN_A 3

#define BACK_RIGHT_PWM 5
#define BACK_RIGHT_BACKWARD A4
#define BACK_RIGHT_FORWARD A5
#define BACK_RIGHT_EN_A 2

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