#ifndef __ODOMATH_H__
#define __ODOMATH_H__

#include <math.h>

#include "../evAPI/Common/include/evNamespace.h"

namespace evAPI {

struct Position {
    double x;      ///< X-coordinate in inches
    double y;      ///< Y-coordinate in inches
    double theta;  ///< Orientation in degrees

    Position(double x = 0, double y = 0, double theta = 0) : x(x), y(y), theta(theta) {}
};

class OdoMath {
  private:
    Position currentPosition;                          ///< Current position and orientation of the robot.
    double prevLeftDist, prevRightDist, prevBackDist;  ///< Previous distances traveled by the left, right, and back wheels.
    double trackWidth;                                 ///< Track width in inches.

  public:
    /**
     * @brief Constructor to initialize the odometry system with track and wheel configurations.
     * @param trackWidth The distance between the left and right wheels in inches.
     */
    OdoMath(double trackWidth)
        // : currentPosition(0, 0, 0), trackWidth(trackWidth), backOffset(backOffset), prevLeftDist(0), prevRightDist(0), prevBackDist(0) {}
        : currentPosition(0, 0, 0), prevLeftDist(0), prevRightDist(0), prevBackDist(0), trackWidth(trackWidth) {}

    /**
     * @brief Sets the initial position of the robot.
     * @param x Initial x-coordinate in inches.
     * @param y Initial y-coordinate in inches.
     * @param theta Initial orientation in degrees.
     */
    void setInitialPosition(double x, double y, double theta);

    /**
     * @brief Updates the robot's position based on distances traveled by each wheel.
     * @param leftDist Current total distance traveled by the left wheel in inches.
     * @param rightDist Current total distance traveled by the right wheel in inches.
     * @param backDist Current total distance traveled by the back wheel in inches.
     */
    void update(double leftDist, double rightDist, double backDist);

    /**
     * @brief Retrieves the current position of the robot.
     * @return Current position as a Position structure.
     */
    Position getCurrentPosition() const {
      return currentPosition;
    }
};

}  // namespace evAPI

#endif  // __ODOMATH_H__