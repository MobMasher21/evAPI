#include "evAPI/odometry/OdoMath.h"
#include <math.h>

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
  double s_l = leftDist - prevLeftDist;
  double s_r = rightDist - prevRightDist;
  double s_b = backDist - prevBackDist;

  this->prevLeftDist = leftDist;
  this->prevRightDist = rightDist;
  this->prevBackDist = backDist;

  double r_h = this->trackWidth;
  // double r_v = this-> ;

  // s_l == s_r a divide by zero will occur
  if (s_l != s_r) {
    // The change in the robots angle
    double theta = (s_r - s_l) / (2 * r_h);
    // The arc length of the movement
    double r_t = (r_h * (s_r + s_l)) / (s_r - s_l);
    
    // Change in x
    double x_1 = r_t * (cos(theta) - 1);
    // Change in y
    double y_1 = r_t * (sin(theta));
    
    // The previous calculations are preformed relative to the robot's orientation at the begining of the movement
    // A rotation by the previous orientation will bring the values into world space relative to the robots starting orientation
    double prev_theta = this->currentPosition.theta;
    double x = x_1 * cos(prev_theta) - y_1 * sin(prev_theta);
    double y = x_1 * sin(prev_theta) + y_1 * cos(prev_theta);
    
    this->currentPosition.x += x;
    this->currentPosition.y += y;
    this->currentPosition.theta += theta;

    printf("x: %f, y: %f, theta: %f\n\n", this->currentPosition.x, this->currentPosition.y, this->currentPosition.theta * 180 / M_PI);
  }
  


  // Debugging outputs to monitor the changes
  // printf("Delta Left: %f, Delta Right: %f, Delta Back: %f\n", dLeft, dRight, dBack);
  //printf("Theta Change: %f degrees, X Change: %f inches, Y Change: %f inches\n", dTheta * (180.0 / M_PI), currentPosition.x, currentPosition.y);
}

}  // namespace evAPI