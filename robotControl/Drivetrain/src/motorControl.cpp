#include "../../../robotControl/Drivetrain/include/Drive.h"

namespace evAPI {
//======================================== private =============================================
/************ motors ************/

void Drive::balanceMotors() {
  activeLeftMotors.clear();
  activeRightMotors.clear();
  
  for (int i = 0; i < motorCount; i++) {
    if (leftMotors[i]->installed() && rightMotors[i]->installed()) {
      activeLeftMotors.push_back(leftMotors[i]);
      activeRightMotors.push_back(rightMotors[i]);
    }
  }
}

/*----- left motors -----*/
void Drive::spinLeftMotors(int speed) {  // spins all motors on the left side
  balanceMotors();
  for (vex::motor* motor : activeLeftMotors) {
    motor->spin(vex::directionType::fwd, speed, vex::pct);
  }
}

void Drive::stopLeftMotors(vex::brakeType type) {  // stop all motors on the left side
  balanceMotors();
  for (vex::motor* motor : activeLeftMotors) {
    motor->stop(type);
  }
}

/*----- right motors -----*/
void Drive::spinRightMotors(int speed) {  // spins all motors on the right side
  balanceMotors();
  for (vex::motor* motor : activeRightMotors) {
    motor->spin(vex::directionType::fwd, speed, vex::pct);
  }
}

void Drive::stopRightMotors(vex::brakeType type) {  // stop all motors on the right side
  balanceMotors();
  for (vex::motor* motor : activeRightMotors) {
    motor->stop(type);
  }
}
}  // namespace evAPI