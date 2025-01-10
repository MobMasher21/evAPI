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
  for (vex::motor* motor : leftMotors) {
    motor->setStopping(mode);
  }

  // Right motors
  for (vex::motor* motor : rightMotors) {
    motor->setStopping(mode);
  }
}

void Drive::motorPortsSetup(evAPI::leftAndRight side, int motorCount, int ports[]) {
  std::vector<vex::motor*>& motorVec = side == evAPI::leftAndRight::LEFT ? leftMotors : rightMotors;
  motorVec.clear();

  if ((this->motorCount != -1) && (this->motorCount != motorCount)) {
    printf("WARNING: left and right motor counts don't match\n");
  }
  
  this->motorCount = motorCount;

  for (int i = 0; i < motorCount; i++) {
    vex::motor* motor = new vex::motor(smartPortLookupTable[ports[i]], currentGear);
    motorVec.push_back(motor);
  }
  
  if (side == evAPI::leftAndRight::LEFT)
  {
    if (leftTracker != nullptr) {
      leftTracker->setEncoderMotor(leftMotors[0]);
    } else {
      leftTracker = new SmartEncoder(leftMotors[0]);
    }
    leftDriveTracker = leftTracker->newTracker();
  } else if (side == evAPI::leftAndRight::RIGHT) {
    if (rightTracker != nullptr) {
      rightTracker->setEncoderMotor(rightMotors[0]);
    } else {
      rightTracker = new SmartEncoder(rightMotors[0]);
    }
    rightDriveTracker = rightTracker->newTracker();
  }
}

void Drive::motorReverseSetup(evAPI::leftAndRight side, int motorCount, bool reverse[]) {
  std::vector<vex::motor*>& motorVec = side == evAPI::leftAndRight::LEFT ? leftMotors : rightMotors;

  if (motorVec.size() != motorCount) {
    printf("ERROR: reverse count doesn't match the number of motors");
    return;
  }
  
  for (int i = 0; i < motorCount; i++) {
    motorVec[i]->setReversed(reverse[i]);
  }
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