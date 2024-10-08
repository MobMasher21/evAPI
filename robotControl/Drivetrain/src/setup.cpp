#include "../include/Drive.h"

namespace evAPI {
//======================================== public =============================================
/*********** setup ***********/
/*----- Base Setup -----*/
void Drive::geartrainSetup(float diameter, int gearIN, int gearOUT) {  // used for setting up the wheel size and gear
  wheelSize = diameter;                                                // ratios for using auto commands
  gearInput = gearIN;
  gearOutput = gearOUT;
  degsPerInch = ((360 / (wheelSize * M_PI)) * (gearOutput / gearInput));
  // ^^ finds the amount of encoder degrees in one inch of movement
  if (isDebugMode) printf("dpi: %f\n", degsPerInch);
}

void Drive::setStoppingMode(vex::brakeType mode) {
  // Left motors
  if (leftMotor1 != nullptr) {
    leftMotor1->setStopping(mode);
  }

  if (leftMotor2 != nullptr) {
    leftMotor2->setStopping(mode);
  }

  if (leftMotor3 != nullptr) {
    leftMotor3->setStopping(mode);
  }

  if (leftMotor4 != nullptr) {
    leftMotor4->setStopping(mode);
  }

  // Right motors
  if (rightMotor1 != nullptr) {
    rightMotor1->setStopping(mode);
  }

  if (rightMotor2 != nullptr) {
    rightMotor2->setStopping(mode);
  }

  if (rightMotor3 != nullptr) {
    rightMotor3->setStopping(mode);
  }

  if (rightMotor4 != nullptr) {
    rightMotor4->setStopping(mode);
  }
}

/*----- motor ports and reverses -----*/
void Drive::setGearbox(vex::gearSetting driveGear) {  // sets gearbox for all motors
  currentGear = driveGear;
}

void Drive::leftPortSetup(int port1) {  // left motor port setup for 2 motor drive
  leftMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  baseMotorCount = 2;
  if (leftTracker != nullptr) {
    leftTracker->setEncoderMotor(leftMotor1);
  } else {
    leftTracker = new SmartEncoder(leftMotor1);
  }
  leftDriveTracker = leftTracker->newTracker();  // creates drive tracker
}

void Drive::leftPortSetup(int port1, int port2) {  // left motor port setup for 4 motor drive
  leftMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  leftMotor2 = new vex::motor(smartPortLookupTable[port2], currentGear);
  baseMotorCount = 4;
  if (leftTracker != nullptr) {
    leftTracker->setEncoderMotor(leftMotor1);
  } else {
    leftTracker = new SmartEncoder(leftMotor1);
  }
  leftDriveTracker = leftTracker->newTracker();  // creates drive tracker
}

void Drive::leftPortSetup(int port1, int port2, int port3) {  // left motor port setup for 6 motor drive
  leftMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  leftMotor2 = new vex::motor(smartPortLookupTable[port2], currentGear);
  leftMotor3 = new vex::motor(smartPortLookupTable[port3], currentGear);
  baseMotorCount = 6;
  if (leftTracker != nullptr) {
    leftTracker->setEncoderMotor(leftMotor1);
  } else {
    leftTracker = new SmartEncoder(leftMotor1);
  }
  leftDriveTracker = leftTracker->newTracker();  // creates drive tracker
}

void Drive::leftPortSetup(int port1, int port2, int port3, int port4) {  // left motor port setup for 8 motor drive
  leftMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  leftMotor2 = new vex::motor(smartPortLookupTable[port2], currentGear);
  leftMotor3 = new vex::motor(smartPortLookupTable[port3], currentGear);
  leftMotor4 = new vex::motor(smartPortLookupTable[port4], currentGear);
  baseMotorCount = 8;
  if (leftTracker != nullptr) {
    leftTracker->setEncoderMotor(leftMotor1);
  } else {
    leftTracker = new SmartEncoder(leftMotor1);
  }
  leftDriveTracker = leftTracker->newTracker();  // creates drive tracker
}

void Drive::rightPortSetup(int port1) {  // right motor port setup for 2 motor drive
  rightMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  if (rightTracker != nullptr) {
    rightTracker->setEncoderMotor(rightMotor1);
  } else {
    rightTracker = new SmartEncoder(rightMotor1);
  }
  rightDriveTracker = rightTracker->newTracker();  // creates drive tracker
}

void Drive::rightPortSetup(int port1, int port2) {  // right motor port setup for 4 motor drive
  rightMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  rightMotor2 = new vex::motor(smartPortLookupTable[port2], currentGear);
  if (rightTracker != nullptr) {
    rightTracker->setEncoderMotor(rightMotor1);
  } else {
    rightTracker = new SmartEncoder(rightMotor1);
  }
  rightDriveTracker = rightTracker->newTracker();  // creates drive tracker
}

void Drive::rightPortSetup(int port1, int port2, int port3) {  // right motor port setup for 6 motor drive
  rightMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  rightMotor2 = new vex::motor(smartPortLookupTable[port2], currentGear);
  rightMotor3 = new vex::motor(smartPortLookupTable[port3], currentGear);
  if (rightTracker != nullptr) {
    rightTracker->setEncoderMotor(rightMotor1);
  } else {
    rightTracker = new SmartEncoder(rightMotor1);
  }
  rightDriveTracker = rightTracker->newTracker();  // creates drive tracker
}

void Drive::rightPortSetup(int port1, int port2, int port3, int port4) {  // right motor port setup for 8 motor drive
  rightMotor1 = new vex::motor(smartPortLookupTable[port1], currentGear);
  rightMotor2 = new vex::motor(smartPortLookupTable[port2], currentGear);
  rightMotor3 = new vex::motor(smartPortLookupTable[port3], currentGear);
  rightMotor4 = new vex::motor(smartPortLookupTable[port4], currentGear);
  if (rightTracker != nullptr) {
    rightTracker->setEncoderMotor(rightMotor1);
  } else {
    rightTracker = new SmartEncoder(rightMotor1);
  }
  rightDriveTracker = rightTracker->newTracker();  // creates drive tracker
}

void Drive::leftReverseSetup(bool reverse1) {  // left motor reverse setup for 2 motor drive
  leftMotor1->setReversed(reverse1);
}

void Drive::leftReverseSetup(bool reverse1, bool reverse2) {  // left motor reverse setup for 4 motor drive
  leftMotor1->setReversed(reverse1);
  leftMotor2->setReversed(reverse2);
}

void Drive::leftReverseSetup(bool reverse1, bool reverse2, bool reverse3) {  // left motor reverse setup for 6 motor drive
  leftMotor1->setReversed(reverse1);
  leftMotor2->setReversed(reverse2);
  leftMotor3->setReversed(reverse3);
}

void Drive::leftReverseSetup(bool reverse1, bool reverse2, bool reverse3, bool reverse4) {  // left motor reverse setup for 8 motor drive
  leftMotor1->setReversed(reverse1);
  leftMotor2->setReversed(reverse2);
  leftMotor3->setReversed(reverse3);
  leftMotor4->setReversed(reverse4);
}

void Drive::rightReverseSetup(bool reverse1) {  // right motor reverse setup for 2 motor drive
  rightMotor1->setReversed(reverse1);
}

void Drive::rightReverseSetup(bool reverse1, bool reverse2) {  // right motor reverse setup for 4 motor drive
  rightMotor1->setReversed(reverse1);
  rightMotor2->setReversed(reverse2);
}

void Drive::rightReverseSetup(bool reverse1, bool reverse2, bool reverse3) {  // right motor reverse setup for 6 motor drive
  rightMotor1->setReversed(reverse1);
  rightMotor2->setReversed(reverse2);
  rightMotor3->setReversed(reverse3);
}

void Drive::rightReverseSetup(bool reverse1, bool reverse2, bool reverse3, bool reverse4) {  // right motor reverse setup for 8 motor drive
  rightMotor1->setReversed(reverse1);
  rightMotor2->setReversed(reverse2);
  rightMotor3->setReversed(reverse3);
  rightMotor4->setReversed(reverse4);
}

/*----- encoder setup -----*/
void Drive::leftEncoderSetup(int port, double wheelSize, bool reverse) {  // setup values for left encoder
  leftEncoder = new vex::rotation(smartPortLookupTable[port], reverse);
  leftEncoderDegsPerInch = (360 / (wheelSize * M_PI));
  if (leftTracker != nullptr) {
    leftTracker->setEncoderRotation(leftEncoder);
  } else {
    leftTracker = new SmartEncoder(leftEncoder);
  }
}

void Drive::rightEncoderSetup(int port, double wheelSize, bool reverse) {  // setup values for right encoder
  rightEncoder = new vex::rotation(smartPortLookupTable[port], reverse);
  rightEncoderDegsPerInch = (360 / (wheelSize * M_PI));
  if (rightTracker != nullptr) {
    rightTracker->setEncoderRotation(rightEncoder);
  } else {
    rightTracker = new SmartEncoder(rightEncoder);
  }
}

void Drive::backEncoderSetup(int port, double wheelSize, bool reverse) {  // setup values for back encoder
  backEncoder = new vex::rotation(smartPortLookupTable[port], reverse);
  backEncoderDegsPerInch = (360 / (wheelSize * M_PI));
  if (backTracker != nullptr) {
    backTracker->setEncoderRotation(backEncoder);
  } else {
    backTracker = new SmartEncoder(backEncoder);
  }
}

/*----- pid setup -----*/
void Drive::setupDrivePID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime) {
  drivePID.setConstants(kp, ki, kd);
  drivePID.setStoppings(minStopError, timeToStop, timeoutTime);
  driveP = kp;
  driveI = ki;
  driveD = kd;
  driveMaxStopError = minStopError;
  driveTimeToStop = timeToStop;
}

void Drive::setupTurnPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime) {
  turnPID.setConstants(kp, ki, kd);
  turnPID.setStoppings(minStopError, timeToStop, timeoutTime);
  turnP = kp;
  turnI = ki;
  turnD = kd;
  turnMaxStopError = minStopError;
  turnTimeToStop = timeToStop;
}

void Drive::setupDriftPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime) {
  driftPID.setConstants(kp, ki, kd);
  driftPID.setStoppings(minStopError, timeToStop, timeoutTime);
  driftP = kp;
  driftI = ki;
  driftD = kd;
  driftMaxStopError = minStopError;
  driftTimeToStop = timeToStop;
}

void Drive::setupArcPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime) {
  arcPID.setConstants(kp, ki, kd);
  arcPID.setStoppings(minStopError, timeToStop, timeoutTime);
  arcP = kp;
  arcI = ki;
  arcD = kd;
  arcMaxStopError = minStopError;
  arcTimeToStop = timeToStop;
}

void Drive::setupArcDriftPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime) {
  arcDriftPID.setConstants(kp, ki, kd);
  arcDriftPID.setStoppings(minStopError, timeToStop, timeoutTime);
  arcDriftP = kp;
  arcDriftI = ki;
  arcDriftD = kd;
  arcDriftMaxStopError = minStopError;
  arcDriftTimeToStop = timeToStop;
}

/*----- inertial setup -----*/
void Drive::setupInertialSensor(int port) {  // sets the port of the inertial sensor
  turnSensor = new vex::inertial(smartPortLookupTable[port]);
}

void Drive::calibrateInertial() {  // calibrate the inertial sensor
  turnSensor->calibrate();
}

bool Drive::isInertialCalibrating() {  // is the inertial sensor calibrating
  return (turnSensor->isCalibrating());
}

void Drive::setDriveHeading(int angle) {  // sets the current heading of the inertial sensor
  turnSensor->setHeading(angle, vex::rotationUnits::deg);
}

}  // namespace evAPI