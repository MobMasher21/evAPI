#include "../include/OdoMath.h"

namespace evAPI {

/**
 * @brief Sets the initial position of the robot.
 * @param x Initial x-coordinate in inches.
 * @param y Initial y-coordinate in inches.
 * @param theta Initial orientation in degrees.
 */
void OdoMath::setInitialPosition(double x, double y, double theta) {
  currentPosition = Position(x, y, theta);
}

/**
 * @brief Updates the robot's position based on distances traveled by each wheel.
 * @param leftDist Current total distance traveled by the left wheel in inches.
 * @param rightDist Current total distance traveled by the right wheel in inches.
 * @param backDist Current total distance traveled by the back wheel in inches.
 */
void OdoMath::update(double leftDist, double rightDist, double backDist) {
  double dLeft = leftDist - prevLeftDist;
  double dRight = rightDist - prevRightDist;
  double dBack = backDist - prevBackDist;

  prevLeftDist = leftDist;
  prevRightDist = rightDist;
  prevBackDist = backDist;

  double dTheta = (dRight - dLeft) / trackWidth;
  double dCenter = (dLeft + dRight) / 2;
  currentPosition.theta += dTheta * (180.0 / M_PI);  // Convert radians to degrees

  currentPosition.x += dCenter * cos(currentPosition.theta * M_PI / 180.0) - dBack * sin(currentPosition.theta * M_PI / 180.0);
  currentPosition.y += dCenter * sin(currentPosition.theta * M_PI / 180.0) + dBack * cos(currentPosition.theta * M_PI / 180.0);

  // Debugging outputs to monitor the changes
  //printf("Delta Left: %f, Delta Right: %f, Delta Back: %f\n", dLeft, dRight, dBack);
  //printf("Theta Change: %f degrees, X Change: %f inches, Y Change: %f inches\n", dTheta * (180.0 / M_PI), currentPosition.x, currentPosition.y);
}

}  // namespace evAPI