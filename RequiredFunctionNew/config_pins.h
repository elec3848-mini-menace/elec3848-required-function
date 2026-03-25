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
const int STRAFE_SPEED = 50;
const int MOTOR_SYNC_DELAY = 250; // Delay before motor sync starts adjusting PWM

const float TICKS_PER_CM = 18.0;   // tune: Number of encoder ticks for 1 cm of wheel travel
const float CM_PER_TICK = 1.0 / TICKS_PER_CM;
const int WALL_EQUAL_TOLERANCE_CM = 1;
const int MOVE_FORWARD_DISTANCE_CM = 25;

// Constants for enhanced wall alignment
const int ALIGNMENT_CONFIRMATION_THRESHOLD = 5;
const int TARGET_WALL_DISTANCE_CM = 22;
const int DISTANCE_TOLERANCE_CM = 1;
const int ALIGN_TURN_DURATION_MS = 150; // Duration for small corrective turns during alignment

// Constants for side wall alignment
const int TARGET_SIDE_WALL_DISTANCE_CM = 60;
const int SIDE_ALIGN_MOVE_DURATION_MS = 150; // Duration for small strafing movements

// Constants for light sensor alignment
const int LIGHT_SENSOR_ALIGN_THRESHOLD = 10; // Difference in sensor readings to consider aligned
const int LIGHT_SENSOR_MOVE_DURATION_MS = 150; // Duration for small strafing movements for light alignment
const int LIGHT_SENSOR_STABLE_COUNT_THRESHOLD = 3; // How many times light sensors must be stable

// Constants for color detection and turning
const int TURN_90_DEGREE_DURATION_MS = 2100; // Approximate duration for a 90-degree turn (needs tuning)
const int COLOR_READ_DELAY_MS = 50; // Delay between color sensor reads

// Wheels
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

// Front Ultrasonic Sensors
#define LEFT_US_TRIG A9
#define LEFT_US_ECHO A8
#define RIGHT_US_TRIG A11
#define RIGHT_US_ECHO A10

// Right-facing Ultrasonic Sensor
#define RIGHT_US_TRIG_SIDE A6
#define RIGHT_US_ECHO_SIDE A0

// Light Sensors
#define LEFT_LIGHT_SENSOR A2
#define RIGHT_LIGHT_SENSOR A3

// TCS3200 Color Sensor
#define COLOR_S0 A12
#define COLOR_S1 A13
#define COLOR_S2 A14
#define COLOR_S3 A15
#define COLOR_OUT A7

// Movement IDs
#define moveForward 0
#define moveBackward 1
#define moveLeft 2
#define moveRight 3
#define moveClockwise 4
#define moveCounterclockwise 5