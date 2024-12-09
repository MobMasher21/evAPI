#include "../include/Drive.h"

namespace evAPI {
bool Drive::isMoving() {
  return leftMotors[0]->isSpinning() || rightMotors[0]->isSpinning();
}

double Drive::getMotorSpeed(vex::turnType side) {
  double wheelVelocity;

  if (side == vex::turnType::left) {
    wheelVelocity = leftMotors[0]->velocity(vex::pct);
  }

  else {
    wheelVelocity = rightMotors[0]->velocity(vex::pct);
  }

  // TODO: Fix this crap
  // Convert the motor velocity to the wheel velocity
  // wheelVelocity *= (gearInput / gearOutput)/*  * (2 * M_PI * wheelSize) */;

  return wheelVelocity;
}

}  // namespace evAPI