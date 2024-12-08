#include "../../../robotControl/Drivetrain/include/Drive.h"

namespace evAPI {
//======================================== private =============================================
/************ motors ************/

void Drive::balanceMotors() {
  vex::motor* leftMotors[4] = {leftMotor1, leftMotor2, leftMotor3, leftMotor4};
  vex::motor* rightMotors[4] = {rightMotor1, rightMotor2, rightMotor3, rightMotor4};

  for (int i = 0; i < baseMotorCount; i++) {
    if (!leftMotors[i]->installed() || !rightMotors[i]->installed()) {
      activeLeftMotors[i] = nullptr;
      activeRightMotors[i] = nullptr;
    }
  }
}

/*----- left motors -----*/
void Drive::spinLeftMotors(int speed) {  // spins all motors on the left side
  balanceMotors();
  for (int i = 0; i < 4; i++) {
    if (activeLeftMotors[i] != nullptr) {
      activeLeftMotors[i]->spin(vex::directionType::fwd, speed, vex::pct);
    }
  }
}

void Drive::stopLeftMotors(vex::brakeType type) {  // stop all motors on the left side
  balanceMotors();
  for (int i = 0; i < 4; i++) {
    if (activeLeftMotors[i] != nullptr) {
      activeLeftMotors[i]->stop(type);
    }
  }
}

/*----- right motors -----*/
void Drive::spinRightMotors(int speed) {  // spins all motors on the right side
  balanceMotors();
  for (int i = 0; i < 4; i++) {
    if (activeRightMotors[i] != nullptr) {
      activeRightMotors[i]->spin(vex::directionType::fwd, speed, vex::pct);
    }
  }
}

void Drive::stopRightMotors(vex::brakeType type) {  // stop all motors on the right side
  balanceMotors();
  for (int i = 0; i < 4; i++) {
    if (activeRightMotors[i] != nullptr) {
      activeRightMotors[i]->stop(type);
    }
  }
}
}  // namespace evAPI