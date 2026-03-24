#include "behavior.h"
#include "config_pins.h"
#include "motor.h"
#include "sensors.h"

void moveForwardDistanceCM(float distanceCM) {
  long targetTicks = (long)(distanceCM * TICKS_PER_CM);

  resetEncoderCounts();
  moveRobot(moveForward, MAIN_SPEED);

  while (getAverageEncoderCount() < targetTicks) {
    motorSyncService();
    delay(1);
  }

  stopRobot();
}

void alignWithWallUsingUltrasonic() {
  Serial.println("Start wall alignment");

  while (true) {
    int leftDist = getUltrasonicCM(0);
    delay(50);
    int rightDist = getUltrasonicCM(1);

    if (leftDist >= 300 || rightDist >= 300 || leftDist <= 0) {
      stopRobot();
      delay(50);
      continue;
    }

    if (abs(leftDist - rightDist) <= WALL_EQUAL_TOLERANCE_CM) {
      stopRobot();
      Serial.println("Wall aligned");
      delay(10000);
      break;
    }

    if (leftDist > rightDist) {
      Serial.println("Turn RIGHT");
      moveRobot(moveClockwise, TURN_SPEED);
      delay(120);
      stopRobot();
    } else if (rightDist > leftDist) {
      Serial.println("Turn LEFT");
      moveRobot(moveCounterclockwise, TURN_SPEED);
      delay(120);
      stopRobot();
    }

    delay(150);
  }
}