#include "../../../robotControl/Drivetrain/include/Drive.h"

namespace evAPI {
//======================================== private =============================================
/************ motors ************/
/*----- left motors -----*/
void Drive::spinLeftMotors(int speed) {  // spins all motors on the left side
  switch (baseMotorCount) {
    case 2:
      leftMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      break;
    case 4:
      leftMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      leftMotor2->spin(vex::directionType::fwd, speed, vex::pct);
      break;
    case 6:
      leftMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      leftMotor2->spin(vex::directionType::fwd, speed, vex::pct);
      leftMotor3->spin(vex::directionType::fwd, speed, vex::pct);
      break;
    case 8:
      leftMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      leftMotor2->spin(vex::directionType::fwd, speed, vex::pct);
      leftMotor3->spin(vex::directionType::fwd, speed, vex::pct);
      leftMotor4->spin(vex::directionType::fwd, speed, vex::pct);
      break;
  }
}

void Drive::stopLeftMotors(vex::brakeType type) {  // stop all motors on the left side
  switch (baseMotorCount) {
    case 2:
      leftMotor1->stop(type);
      break;
    case 4:
      leftMotor1->stop(type);
      leftMotor2->stop(type);
      break;
    case 6:
      leftMotor1->stop(type);
      leftMotor2->stop(type);
      leftMotor3->stop(type);
      break;
    case 8:
      leftMotor1->stop(type);
      leftMotor2->stop(type);
      leftMotor3->stop(type);
      leftMotor4->stop(type);
      break;
  }
}

/*----- right motors -----*/
void Drive::spinRightMotors(int speed) {  // spins all motors on the right side
  switch (baseMotorCount) {
    case 2:
      rightMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      break;
    case 4:
      rightMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      rightMotor2->spin(vex::directionType::fwd, speed, vex::pct);
      break;
    case 6:
      rightMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      rightMotor2->spin(vex::directionType::fwd, speed, vex::pct);
      rightMotor3->spin(vex::directionType::fwd, speed, vex::pct);
      break;
    case 8:
      rightMotor1->spin(vex::directionType::fwd, speed, vex::pct);
      rightMotor2->spin(vex::directionType::fwd, speed, vex::pct);
      rightMotor3->spin(vex::directionType::fwd, speed, vex::pct);
      rightMotor4->spin(vex::directionType::fwd, speed, vex::pct);
      break;
  }
}

void Drive::stopRightMotors(vex::brakeType type) {  // stop all motors on the right side
  switch (baseMotorCount) {
    case 2:
      rightMotor1->stop(type);
      break;
    case 4:
      rightMotor1->stop(type);
      rightMotor2->stop(type);
      break;
    case 6:
      rightMotor1->stop(type);
      rightMotor2->stop(type);
      rightMotor3->stop(type);
      break;
    case 8:
      rightMotor1->stop(type);
      rightMotor2->stop(type);
      rightMotor3->stop(type);
      rightMotor4->stop(type);
      break;
  }
}
}  // namespace evAPI