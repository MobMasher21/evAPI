#ifndef DRIVERCONTROL_H_
#define DRIVERCONTROL_H_

#include "evAPI/common/evAPIBasicConfig.h"
#include "evAPI/common/evNamespace.h"
#include "evAPI/drivetrain/Drive.h"

// using namespace vex;

namespace evAPI {
class DriverBaseControl {
  public:
    /**
     * @brief Construct a new Driver Control object
     *
     * @param controllerIN A vex controller
     * @param driveTypeIN The type of controlling that will be used
     * @param drivetrainIN The drivetrain that will be controlled
     */
    DriverBaseControl(vex::controller* controllerIN, driveMode driveTypeIN, Drive* drivetrainIN);

    /**
     * @brief Set the Handicaps
     *
     * @param driveCap Percent power of the drive stick
     * @param turnCap Percent power of the turn stick
     */
    void setHandicaps(double driveCap, double turnCap);
    void setHandicaps(double driveCap);

    /**
     * @brief Set the primary stick for drive types that need it
     *
     * @param primaryStickIN
     */
    void setPrimaryStick(joystickType primaryStickIN);

    /**
     * @brief Called in the main driver contorl loop to drive the base
     *
     */
    void driverLoop();

  private:
    vex::controller* vexController;
    driveMode driverType;
    Drive* drivetrain;
    double driveHandicap = 1;
    double turnHandicap = 1;
    joystickType primaryStick = leftStick;
};
}  // namespace evAPI

#endif  // DRIVERCONTROL_H_