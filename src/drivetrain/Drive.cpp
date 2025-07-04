#include "evAPI/drivetrain/Drive.h"

namespace evAPI {
Drive* threadReference;

//======================================== public =============================================
/****** constructors ******/
Drive::Drive(void) {
  threadReference = this;
}

Drive::Drive(vex::gearSetting driveGear) {
  currentGear = driveGear;
  threadReference = this;
}

/****** debug ******/
void Drive::setDebugState(bool mode) {  // allows you to toggle debug mode
  isDebugMode = mode;
}

void Drive::printAllEncoderData() {  // prints all 3 encoder values to the terminal
  printf("drive action (l, r): %f, %f\n", leftTracker->readTrackerPosition(leftDriveTracker), rightTracker->readTrackerPosition(rightDriveTracker));
  printf("odo action (l, r): %f, %f\n", leftTracker->readTrackerPosition(leftOdoTracker), rightTracker->readTrackerPosition(rightOdoTracker));
}

/************ movement ************/
/*----- manual movement -----*/
void Drive::spinBase(int leftSpeed, int rightSpeed) {
  spinLeftMotors(leftSpeed);
  spinRightMotors(rightSpeed);
}

void Drive::stopRobot() {  // stops robot with type coast
  stopLeftMotors(vex::brakeType::coast);
  stopRightMotors(vex::brakeType::coast);
}

void Drive::stopRobot(vex::brakeType stoppingMode) {  // stops robot with given type
  stopLeftMotors(stoppingMode);
  stopRightMotors(stoppingMode);
}

/*----- automatic -----*/
void Drive::setDriveSpeed(int speed) {  // sets the drive speed for when one is not entered
  driveSpeed = speed;
}

void Drive::setTurnSpeed(int speed) {  // sets the turn speed for when one is not entered
  turnSpeed = speed;
}

void Drive::setArcTurnSpeed(int speed) {  // sets the arc turn speed for when one is not entered
  arcTurnSpeed = speed;
}

void Drive::setDriveBaseWidth(double width) {  // sets the distance between the two wheels for arc turns
  driveBaseWidth = width;
}

void Drive::driveForward(double distance, int speed, DriveCallbackUserData callback, void* userdata) {  // enter a distance and speed to go forward
  //*setup of all variables*
  double leftPosition;       // angle of left encoder
  double rightPosition;      // angle of right encoder
  int averagePosition;       // average position of both encoders
  int error;                 // desired value - sensor value
  int driftError;            // difference between left - right
  int desiredValue;          // angle of rotation sensor that we want
  bool isPIDRunning = true;  // is true as the PID is running
  int moveSpeed;             // the speed the motors are set to every cycle
  int driftPower;            // output of the drift PID
  drivePID.setTotalError(0);
  driftPID.setTotalError(0);
  drivePID.resetTimeout();

  //*checks to see if you have encoders and then sets the desired angle of the pid*
  if (leftEncoder) {
    desiredValue = distance * leftEncoderDegsPerInch;
  } else {
    desiredValue = distance * degsPerInch;
  }
  if (isDebugMode) printf("desiredValue: %i\n", desiredValue);

  //*resets encoders*
  leftTracker->resetTrackerPosition(leftDriveTracker);
  rightTracker->resetTrackerPosition(rightDriveTracker);
  if (isDebugMode) printf("rightAngle: %f\n", leftTracker->readTrackerPosition(leftDriveTracker));
  if (isDebugMode) printf("leftAngle: %f\n", rightTracker->readTrackerPosition(rightDriveTracker));

  //*print debug header*
  if (isDebugMode) printf("position, error, moveSpeed\n");

  //*main PID loop*
  while (isPIDRunning) {
    //*get encoder positions*
    leftPosition = leftTracker->readTrackerPosition(leftDriveTracker);
    rightPosition = rightTracker->readTrackerPosition(rightDriveTracker);
    averagePosition = (leftPosition + rightPosition) / 2;
    driftError = leftPosition - rightPosition;

    //*calculate error for this cycle*
    error = desiredValue - averagePosition;

    //*adding all tunning values*
    moveSpeed = drivePID.compute(error);
    driftPower = driftPID.compute(driftError);

    //*speed cap
    if (moveSpeed > speed) moveSpeed = speed;
    if (moveSpeed < -speed) moveSpeed = -speed;

    // Populate struct that will be passed to the user callback
    bool stop = false;
    bool abort = false;
    auto info = DriveInfo{
        &desiredValue,
        &error,
        &speed,
        &moveSpeed,
        &driftPower,
        leftPosition,
        rightPosition,
        averagePosition,
        &stop,
        &abort};

    if (callback != nullptr) {
      callback(info, userdata);
    }

    if (stop) break;
    if (abort) return;

    //*setting motor speeds*
    spinBase(moveSpeed - driftPower, moveSpeed + driftPower);

    //*stopping code*
    if (drivePID.isSettled()) {
      isPIDRunning = false;
    }

    //*print debug data*
    if (isDebugMode) {
      printf("%i, ", averagePosition);
      printf("%i, ", error);
      printf("%i\n", moveSpeed);
    }

    //*wait to avoid overloading*
    vex::task::sleep(20);
  }
  stopRobot(vex::brakeType::brake);
}

void Drive::driveForward(double distance, int speed, DriveCallback callback) {
  if (callback != nullptr) {
    auto wrapper = [](DriveInfo info, void* userdata) { ((DriveCallback)userdata)(info); };
    driveForward(distance, speed, wrapper, (void*)callback);
  } else {
    driveForward(distance, speed, nullptr, nullptr);
  }
}

void Drive::driveForward(double distance, int speed) {
  driveForward(distance, speed, nullptr);
}

void Drive::driveForward(double distance, DriveCallbackUserData callback, void* userdata) {  // enter a distance to go forward
  driveForward(distance, driveSpeed, callback, userdata);
}

void Drive::driveForward(double distance, DriveCallback callback) {  // enter a distance to go forward
  driveForward(distance, driveSpeed, callback);
}

void Drive::driveForward(double distance) {  // enter a distance to go forward
  driveForward(distance, driveSpeed);
}

void Drive::driveBackward(double distance, int speed) {  // enter a distance and speed to go backward
  driveForward(-distance, speed);
}

void Drive::driveBackward(double distance) {  // enter a distance to go backward
  driveForward(-distance, driveSpeed);
}

void Drive::turnToHeading(int angle, int speed, TurnCallbackUserData callback, void* userdata) {  // enter an angle and speed to turn
  //*setup of all variables*
  leftAndRight turnDirection;
  int currentHeading;
  int error;                 // desired value - sensor value
  int desiredValue;          // angle of rotation sensor that we want
  bool isPIDRunning = true;  // is true as the PID is running
  int moveSpeed;             // the speed the motors are set to every cycle
  turnPID.setTotalError(0);
  turnPID.resetTimeout();

  //*checks to see if you have an inertial and then sets the desired angle of the pid*
  if (turnSensor) {
    desiredValue = angle;
    turnDirection = findDir(turnSensor->heading(vex::rotationUnits::deg), desiredValue);
  } else {
    return;
  }

  //*print debug header*
  if (isDebugMode) printf("error, moveSpeed");

  //*main PID loop*
  while (isPIDRunning) {
    //*get heading*
    currentHeading = turnSensor->heading(vex::rotationUnits::deg);

    //*calculate error for this cycle*
    error = turnError(turnDirection, currentHeading, desiredValue);

    //*adding all tunning values*
    moveSpeed = turnPID.compute(error);

    //*speed cap
    if (moveSpeed > speed) moveSpeed = speed;
    if (moveSpeed < -speed) moveSpeed = -speed;
    
    // Populate struct that will be passed to the user callback
    bool stop = false;
    bool abort = false;
    auto info = TurnInfo{
        &desiredValue,
        &error,
        &speed,
        &moveSpeed,
        &stop,
        &abort};

    if (callback != nullptr) {
      callback(info, userdata);
    }

    if (stop) break;
    if (abort) return;

    //*setting motor speeds*
    if (turnDirection == LEFT) {
      spinBase(-moveSpeed, moveSpeed);
    } else if (turnDirection == RIGHT) {
      spinBase(moveSpeed, -moveSpeed);
    }

    //*stopping code*
    if (turnPID.isSettled()) {
      isPIDRunning = false;
    }

    //*print debug data*
    if (isDebugMode) {
      printf("%i, ", error);
      printf("%i\n", moveSpeed);
    }

    //*wait to avoid overloading*
    vex::task::sleep(20);
  }

  stopRobot(vex::brakeType::brake);
}

void Drive::turnToHeading(int angle, int speed, TurnCallback callback) {
  if (callback != nullptr) {
    auto wrapper = [](TurnInfo info, void* userdata) { ((TurnCallback)userdata)(info); };
    turnToHeading(angle, speed, wrapper, (void*)callback);
  } else {
    turnToHeading(angle, speed, nullptr, nullptr);
  }
}

void Drive::turnToHeading(int angle, int speed) {
  turnToHeading(angle, speed, nullptr);
}

void Drive::turnToHeading(int angle, TurnCallbackUserData callback, void* userdata) {
  turnToHeading(angle, turnSpeed, callback, userdata);
}

void Drive::turnToHeading(int angle, TurnCallback callback) {
  turnToHeading(angle, turnSpeed, callback);
}

void Drive::turnToHeading(int angle) {  // enter an angle to turn
  turnToHeading(angle, turnSpeed);
}

void Drive::turnFor(double angle, vex::turnType direction, int speed) {
  double targetHeading;

  //*Limit the angle to be between 0-360
  while (angle >= 360) angle -= 360;
  while (angle < 0) angle += 360;

  //*Turn in the proper direction
  if (direction == vex::turnType::left) {
    // Calculate the target heading
    targetHeading = turnSensor->heading() - angle;
    while (targetHeading >= 360) targetHeading -= 360;
    while (targetHeading < 0) targetHeading += 360;

    // Turn to the desired heading
    turnToHeading(targetHeading, speed);
  }

  else {
    // Calculate the target heading
    targetHeading = turnSensor->heading() + angle;
    while (targetHeading >= 360) targetHeading -= 360;
    while (targetHeading < 0) targetHeading += 360;

    // Turn to the desired heading
    turnToHeading(targetHeading, speed);
  }
}

void Drive::turnFor(double angle, vex::turnType direction) {
  turnFor(angle, direction, turnSpeed);
}

void Drive::arcTurn(double radius, vex::turnType direction, int angle, int speed) {  // turns in an arc
  //*setup of all variables*
  double leftPosition;       // angle of left encoder
  double rightPosition;      // angle of right encoder
  int error;                 // desired value - sensor value
  int driftError;            // difference between wheel power ratio - current power ratio (1000x for math purposes)
  int desiredValue;          // angle of rotation sensor that we want
  double outerDistance;      // length of the outer arc of the turn
  double innerDistance;      // length of the inner arc of the turn
  double wheelPowerRatio;    // ratio of length between outer and inner wheel
  bool isPIDRunning = true;  // is true as the PID is running
  int moveSpeed;             // the speed the motors are set to every cycle
  int driftPower;            // output of the drift PID
  double slowStart = 0;
  arcPID.setTotalError(0);
  arcDriftPID.setTotalError(0);
  arcPID.resetTimeout();

  outerDistance = (((radius + (driveBaseWidth / 2)) * 2) * M_PI) * ((double)angle / 360);  // inches of outer arc
  innerDistance = (((radius - (driveBaseWidth / 2)) * 2) * M_PI) * ((double)angle / 360);  // inches of inner arc
  wheelPowerRatio = innerDistance / outerDistance;

  //*resets encoders*
  leftTracker->resetTrackerPosition(leftDriveTracker);
  rightTracker->resetTrackerPosition(rightDriveTracker);
  if (isDebugMode) printf("rightAngle: %f\n", leftTracker->readTrackerPosition(leftDriveTracker));
  if (isDebugMode) printf("leftAngle: %f\n", rightTracker->readTrackerPosition(rightDriveTracker));

  //*print debug header*
  if (isDebugMode) printf("outerDistance: %f\n", outerDistance);
  if (isDebugMode) printf("innerDistance: %f\n", innerDistance);
  if (isDebugMode) printf("driveBaseWidth: %f\n", driveBaseWidth);

  if (direction == vex::left) {
    if (rightEncoder) {
      desiredValue = outerDistance * rightEncoderDegsPerInch;
    } else {
      desiredValue = outerDistance * degsPerInch;
    }
    if (isDebugMode) printf("desiredValue: %i\n", desiredValue);
    if (isDebugMode) printf("wheelPowerRatio = %f\n", wheelPowerRatio);

    if (isDebugMode) printf("position, error, moveSpeed\n");
    //*main PID loop*
    while (isPIDRunning) {
      //*get encoder positions*
      leftPosition = leftTracker->readTrackerPosition(leftDriveTracker);
      rightPosition = rightTracker->readTrackerPosition(rightDriveTracker);

      //*calculate error for this cycle*
      error = desiredValue - rightPosition;
      driftError = (wheelPowerRatio - (leftPosition / rightPosition)) * 1000;  // desired ratio - current ratio

      //*adding all tunning values*
      moveSpeed = arcPID.compute(error);
      driftPower = arcDriftPID.compute(driftError);

      //*speed cap
      if (moveSpeed > speed) moveSpeed = speed;
      if (moveSpeed < -speed) moveSpeed = -speed;

      //*setting motor speeds*
      spinBase((moveSpeed * wheelPowerRatio) + (driftPower / 1000), moveSpeed);  // outer wheel always moves at same speed and inner wheel changes to adapt

      //*stopping code*
      if (arcPID.isSettled()) {
        isPIDRunning = false;
      }

      //*print debug data*
      if (isDebugMode) {
        printf("%i, ", (int)floor(leftPosition));
        printf("%i, ", error);
        printf("%i\n", moveSpeed);
      }

      //*wait to avoid overloading*
      vex::task::sleep(20);
    }
  } else if (direction == vex::right) {
    if (leftEncoder) {
      desiredValue = outerDistance * leftEncoderDegsPerInch;
    } else {
      desiredValue = outerDistance * degsPerInch;
    }
    if (isDebugMode) printf("desiredValue: %i\n", desiredValue);
    if (isDebugMode) printf("wheelPowerRatio = %f\n", wheelPowerRatio);

    if (isDebugMode) printf("position, error, moveSpeed\n");
    //*main PID loop*
    while (isPIDRunning) {
      if (slowStart < 1) {
        slowStart += 0.005;
      }
      //*get encoder positions*
      leftPosition = leftTracker->readTrackerPosition(leftDriveTracker);
      rightPosition = rightTracker->readTrackerPosition(rightDriveTracker);

      //*calculate error for this cycle*
      error = desiredValue - leftPosition;
      driftError = (wheelPowerRatio - (rightPosition / leftPosition)) * 1000;  // desired ratio - current ratio

      //*adding all tunning values*
      moveSpeed = arcPID.compute(error) * slowStart;
      driftPower = arcDriftPID.compute(driftError);

      //*speed cap
      if (moveSpeed > speed) moveSpeed = speed;
      if (moveSpeed < -speed) moveSpeed = -speed;

      //*setting motor speeds*
      spinBase(moveSpeed, (moveSpeed * wheelPowerRatio) + (driftPower / 1000));  // outer wheel always moves at same speed and inner wheel changes to adapt

      //*stopping code*
      if (arcPID.isSettled()) {
        isPIDRunning = false;
      }

      //*print debug data*
      if (isDebugMode) {
        printf("%i, ", (int)floor(rightPosition));
        printf("%i, ", error);
        printf("%i\n", moveSpeed);
      }

      //*wait to avoid overloading*
      vex::task::sleep(20);
    }
  }
  stopRobot(vex::brakeType::brake);
}

void Drive::arcTurn(double radius, vex::turnType direction, int angle) {
  arcTurn(radius, direction, angle, arcTurnSpeed);
}

//======================================== private =============================================
/****** formulas ******/
leftAndRight Drive::findDir(int startingAngle, int endingAngle) {
  leftAndRight output;
  int leftDegs;
  int rightDegs;
  int x;  // starting angle + 360
  // left turn
  x = startingAngle + 360;
  if (x - endingAngle > 360) {
    x = startingAngle;
  }
  leftDegs = x - endingAngle;
  // right turn
  x = endingAngle + 360;
  if (x - startingAngle > 360) {
    x = endingAngle;
  }
  rightDegs = x - startingAngle;
  // return lines
  if (leftDegs > rightDegs) {
    output = RIGHT;
  } else {
    output = LEFT;
  }
  return (output);
}

int Drive::turnError(leftAndRight direction, int startAngle, int endAngle) {
  int turnError = 0;
  if (direction == LEFT) {
    //===============================LEFT
    if (((startAngle + 360) - endAngle) > 180) {
      if (startAngle - endAngle < 180) {
        turnError = startAngle - endAngle;
      } else if (startAngle - endAngle > 180) {
        turnError = (startAngle - endAngle) - 360;
      } else {
        turnError = 180;
      }
    } else if (((startAngle + 360) - endAngle) < 180) {
      turnError = (startAngle + 360) - endAngle;
    } else {
      turnError = 180;
    }

  } else if (direction == RIGHT) {
    //===============================RIGHT
    if (((endAngle + 360) - startAngle) > 180) {
      if (endAngle - startAngle < 180) {
        turnError = endAngle - startAngle;
      } else if (endAngle - startAngle > 180) {
        turnError = (endAngle - startAngle) - 360;
      } else {
        turnError = 180;
      }
    } else if (((endAngle + 360) - startAngle) < 180) {
      turnError = (endAngle + 360) - startAngle;
    } else {
      turnError = 180;
    }
    //===============================
  }

  return (turnError);
}

vex::motor* Drive::getLeftMotor(int index) {
  return index < leftMotors.size() ? leftMotors[index] : nullptr;
}

vex::motor* Drive::getRightMotor(int index) {
  return index < rightMotors.size() ? rightMotors[index] : nullptr;
}
}  // namespace evAPI