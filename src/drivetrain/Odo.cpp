#include "evAPI/drivetrain/Drive.h"

namespace evAPI {
extern Drive* threadReference;

void hiddenOdoThreadFunction();  // function for odo thread

void Drive::setupOdometry(double trackWidth) {
  if (trackWidth == 0) trackWidth = driveBaseWidth;
  odoTracker = new OdoMath(trackWidth);
  leftOdoTracker = leftTracker->newTracker();    // creates right odo tracker
  rightOdoTracker = rightTracker->newTracker();  // creates right odo tracker
  backOdoTracker = backTracker->newTracker();    // creates back odo tracker
}

void Drive::odoThreadFunction() {  // command only called by odo thread loop
  double leftInches = leftTracker->readTrackerPosition(leftOdoTracker) / leftEncoderDegsPerInch;
  double rightInches = rightTracker->readTrackerPosition(rightOdoTracker) / rightEncoderDegsPerInch;
  double backInches = backTracker->readTrackerPosition(backOdoTracker) / backEncoderDegsPerInch;
  odoTracker->update(leftInches, rightInches, backInches);
  if (isDebugMode) {
    printf("x:%f, y:%f, ", odoTracker->getCurrentPosition().x, odoTracker->getCurrentPosition().y);
    printf("deg:%f\n", odoTracker->getCurrentPosition().theta);
  }
}

Position Drive::returnOdoPosition() {  // returns the position structure that stores the odometry data
  return (odoTracker->getCurrentPosition());
}

void Drive::startOdoThread() {  // starts the odo tracking thread
  odoTracker->setInitialPosition(0, 0, 0);
  leftTracker->resetAll();
  rightTracker->resetAll();
  backTracker->resetAll();
  odoThread = new vex::thread(hiddenOdoThreadFunction);
}

void hiddenOdoThreadFunction() {  // function for odo thread
  while (1) {
    threadReference->odoThreadFunction();
    vex::this_thread::sleep_for(20);
  }
}
}  // namespace evAPI
