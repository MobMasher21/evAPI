#ifndef __DRIVE_H__
#define __DRIVE_H__

#include <array>

#include "evAPI/common/PID.h"
#include "evAPI/common/generalFunctions.h"
#include "evAPI/odometry/OdoMath.h"
#include "SmartEncoder.h"

/**
 * ! Remember to keep code well documented
 * * Item order is only and idea
 * * Some things can be done simultaneously
 * //TODO: write setup code for pid constants
 * //TODO: write pid for driving
 * //TODO: write pid for turning
 * //TODO: write setup code of inertial sensor
 * //TODO: write drift contorl for driving
 * //TODO: write automatic controller Configuration for driver control
 * TODO: write odometry position tracking
 * TODO: write drive to point for odometry
 * TODO: write path finding for odometry
 * TODO: integrate vision tracking
 * TODO: Add functions to get the current state of the drive base, like its velocity
 *
 */

#define DRIVE_TRACKER 0
#define ODO_TRACKER   1


namespace evAPI {

struct DriveInfo {
  // mutable values
  int *desiredValue;
  int *error;
  int *desiredSpeed;
  int *moveSpeed;
  int *driftPower;

  // immutable values
  double leftPosition;
  double rightPosition;
  int averagePosition;

  // flags
  bool *stop; // breaks out of the PID loop and stops the base
  bool *abort; // immediately returns from the drive command without stopping the base
};
using DriveCallback = void(*)(DriveInfo);
using DriveCallbackUserData = void(*)(DriveInfo, void*);

struct TurnInfo {
  // mutable values
  int *desiredValue;
  int *error;
  int *desiredSpeed;
  int *moveSpeed;

  // flags
  bool *stop; // breaks out of the PID loop and stops the base
  bool *abort; // immediately returns from the drive command without stopping the base
};
using TurnCallback = void(*)(TurnInfo);
using TurnCallbackUserData = void(*)(TurnInfo, void*);

struct ArcInfo {
  // mutable values
  int *desiredValue;
  int *error;
  int *desiredSpeed;
  int *moveSpeed;
  int *driftPower;

  // immutable values
  double leftPosition;
  double rightPosition;
  int averagePosition;

  // flags
  bool *stop; // breaks out of the PID loop and stops the base
  bool *abort; // immediately returns from the drive command without stopping the base
};
using ArcCallback = void(*)(ArcInfo);
using ArcCallbackUserData = void(*)(ArcInfo, void*);

class Drive {
  public:
    /****** constructors ******/

    /**
     * @brief Creates a drivetrain object.
     */
    Drive(void);

    /**
     * @brief Creates a drivetrain object.
     * @param driveGear Optional. Sets the cartage type for all the base motors.
     */
    Drive(vex::gearSetting driveGear);

    /****** debug ******/

    /**
     * @brief Controls debug mode, which prints out information when driving.
     * @param mode Enables or disables debug mode.
     */
    void setDebugState(bool mode);

    /**
     * @brief Prints all the data from the encoders to the terminal.
     */
    void printAllEncoderData();

    /*********** setup ***********/
    /*----- Base Setup -----*/

    /**
     * @brief Sets up the geartrain values for the base.
     * @param diameter The diameter of the wheels in inches.
     * @param gearIN The number of teeth on the gears connected to the motors.
     * @param gearOUT The number of teeth on the gears connected to the wheels.
     */
    void geartrainSetup(float diameter, int gearIN, int gearOUT);

    /**
     * @brief Sets the stopping mode for the base.
     * @param mode The stopping mode.
     */
    void setStoppingMode(vex::brakeType mode);

    /*----- motor ports and reverses -----*/

    /**
     * @brief Sets ports for the left side
     */
    template <typename... Args>
    void leftPortSetup(int port, Args... ports) {
      int portArray[sizeof...(Args) + 1] = {port, ports...};
      motorPortsSetup(evAPI::leftAndRight::LEFT, sizeof...(Args) + 1, portArray);
    }

    /**
     * @brief Sets ports for the right side
     */
    template <typename... Args>
    void rightPortSetup(int port, Args... ports) {
      int portArray[sizeof...(Args) + 1] = {port, ports...};
      motorPortsSetup(evAPI::leftAndRight::RIGHT, sizeof...(Args) + 1, portArray);
    }

    /**
     * @brief Sets reverse status for the left side
     */
    template <typename... Args>
    void leftReverseSetup(bool reverse, Args... reverses) {
      bool reverseArray[sizeof...(Args) + 1] = {reverse, reverses...};
      motorReverseSetup(evAPI::leftAndRight::LEFT, sizeof...(Args) + 1, reverseArray);
    }

    /**
     * @brief Sets reverse status for the left side
     */
    template <typename... Args>
    void rightReverseSetup(bool reverse, Args... reverses) {
      bool reverseArray[sizeof...(Args) + 1] = {reverse, reverses...};
      motorReverseSetup(evAPI::leftAndRight::RIGHT, sizeof...(Args) + 1, reverseArray);
    }

    /*----- encoder setup -----*/

    /**
     * @brief Sets up the left encoder on the base.
     * @param port The port the encoder is in.
     * @param wheelSize The size of the wheel the encoder is connected to in inches.
     * @param reverse Optional. Controls if the encoder is reversed or not.
     */
    void leftEncoderSetup(int port, double wheelSize, bool reverse = false);

    /**
     * @brief Sets up the right encoder on the base.
     * @param port The port the encoder is in.
     * @param wheelSize The size of the wheel the encoder is connected to in inches.
     * @param reverse Optional. Controls if the encoder is reversed or not.
     */
    void rightEncoderSetup(int port, double wheelSize, bool reverse = false);

    /**
     * @brief Sets up the back encoder on the base
     *
     * @param port The port the encoder is in.
     * @param wheelSize The size of the wheel the encoder is connected to in inches.
     * @param reverse Optional. Controls if the encoder is reversed or not.
     */
    void backEncoderSetup(int port, double wheelSize, bool reverse = false);

    /**
     * @brief Resets all position data for the left encoder.
     */
    void leftFullReset();

    /**
     * @brief Resets all position data for the right encoder.
     */
    void rightFullReset();

    /*----- pid setup -----*/

    /**
     * @brief Sets up the PID controller for driving straight.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param startI The startI value for the PID
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupDrivePID(double kp, double ki, double kd, int startI, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for driving straight.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupDrivePID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for point turning.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param startI The startI value for the PID
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupTurnPID(double kp, double ki, double kd, int startI, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for point turning.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupTurnPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for drift control when driving straight.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param startI The startI value for the PID
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupDriftPID(double kp, double ki, double kd, int startI, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for drift control when driving straight.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupDriftPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for arc turning.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param startI The startI value for the PID
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupArcPID(double kp, double ki, double kd, int startI, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for arc turning.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupArcPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for drift control when arc turning.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param startI The startI value for the PID
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupArcDriftPID(double kp, double ki, double kd, int startI, int minStopError, int timeToStop, int timeoutTime);

    /**
     * @brief Sets up the PID controller for drift control when arc turning.
     * @param kp The proportional value for the PID.
     * @param ki The integral value for the PID.
     * @param kd The derivative value for the PID.
     * @param minStopError The minimum error required for the PID to finish.
     * @param timeToStop The amount of cycles the PID needs to run for with the error being less than
     *                   minStopError for the PID to finish.
     * @param timeoutTime The amount of cycles the PID needs to take before it times out and exits.
     */
    void setupArcDriftPID(double kp, double ki, double kd, int minStopError, int timeToStop, int timeoutTime);

    /*----- inertial setup -----*/

    /**
     * @brief Sets up an inertial sensor.
     * @param port The port the sensor is in.
     */
    void setupInertialSensor(int port);

    /**
     * @brief Calibrates the inertial sensor.
     */
    void calibrateInertial();

    /**
     * @brief Checks if the inertial sensor is calibrating.
     * @returns True if the sensor is calibrating.
     */
    bool isInertialCalibrating();

    /**
     * @brief Sets the current heading of the base.
     * @param angle The angle the robot is facing from 0-360 degrees.
     */
    void setDriveHeading(int angle);

    /************ movement ************/

    /*----- manual movement -----*/
    /**
     * @brief Spins the motors in each side with a given speed in percent.
     * @param leftSpeed The speed of the left motors.
     * @param rightSpeed The speed of the right motors.
     */
    void spinBase(int leftSpeed, int rightSpeed);

    /**
     * @brief Stops the base motors with a brake type of coast.
     */
    void stopRobot();

    /**
     * @brief Stops the base motors with a specified stopping mode.
     * @param stoppingMode The was the base motors will stop.
     */
    void stopRobot(vex::brakeType stoppingMode);

    /*----- automatic -----*/

    /**
     * @brief Sets the default drive speed when using the drive functions.
     * @param speed The top speed of the base in percent.
     */
    void setDriveSpeed(int speed);

    /**
     * @brief Sets the default point turn speed when using the turn functions.
     * @param speed The top speed of the base in percent.
     */
    void setTurnSpeed(int speed);

    /**
     * @brief Sets the default arc turn speed when using the turn functions.
     * @param speed The top speed of the base in percent.
     * @warning If using a base that easily moves from side to side, it is recommended that you
     *          keep this value less than 50. Otherwise, your robot may drift excessively.
     */
    void setArcTurnSpeed(int speed);

    /**
     * @brief Sets the width of the base.
     * @param width The width of the base in inches.
     */
    void setDriveBaseWidth(double width);

    /**
     * @brief Drives the robot forward.
     * @param distance The distance to drive in inches.
     * @param speed Optional. The top speed to drive at.
     */
    void driveForward(double distance, int speed, DriveCallbackUserData callback, void* userdata);
    
    /**
     * @brief Drives the robot forward.
     * @param distance The distance to drive in inches.
     * @param speed Optional. The top speed to drive at.
     */
    void driveForward(double distance, int speed, DriveCallback callback);

    /**
     * @brief Drives the robot forward.
     * @param distance The distance to drive in inches.
     * @param speed Optional. The top speed to drive at.
     */
    void driveForward(double distance, int speed);

    /**
     * @brief Drives the robot forward.
     * @param distance The distance to drive in inches.
     */
    void driveForward(double distance, DriveCallbackUserData callback, void* userdata);

    /**
     * @brief Drives the robot forward.
     * @param distance The distance to drive in inches.
     */
    void driveForward(double distance, DriveCallback callback);

    /**
     * @brief Drives the robot forward.
     * @param distance The distance to drive in inches.
     */
    void driveForward(double distance);

    /**
     * @brief Drives the robot backward.
     * @param distance The distance to drive in inches.
     * @param speed Optional. The top speed to drive at.
     */
    void driveBackward(double distance, int speed);

    /**
     * @brief Drives the robot backward.
     * @param distance The distance to drive in inches.
     */
    void driveBackward(double distance);

    /**
     * @brief Turns the robot to a specified heading.
     * @param angle The heading to turn to in degrees.
     * @param speed Optional. The top speed to turn at.
     */
    void turnToHeading(int angle, int speed, TurnCallbackUserData callback, void* userdata);

    /**
     * @brief Turns the robot to a specified heading.
     * @param angle The heading to turn to in degrees.
     * @param speed Optional. The top speed to turn at.
     */
    void turnToHeading(int angle, int speed, TurnCallback callback);

    /**
     * @brief Turns the robot to a specified heading.
     * @param angle The heading to turn to in degrees.
     * @param speed Optional. The top speed to turn at.
     */
    void turnToHeading(int angle, int speed);

    /**
     * @brief Turns the robot to a specified heading.
     * @param angle The heading to turn to in degrees.
     */
    void turnToHeading(int angle, TurnCallbackUserData callback, void* userdata);

    /**
     * @brief Turns the robot to a specified heading.
     * @param angle The heading to turn to in degrees.
     */
    void turnToHeading(int angle, TurnCallback callback);

    /**
     * @brief Turns the robot to a specified heading.
     * @param angle The heading to turn to in degrees.
     */
    void turnToHeading(int angle);

    /**
     * @brief Turns the robot a specified amount.
     * @param angle The amount of degrees to turn.
     * @param direction The direction of the turn.
     * @param speed Optional. The top speed of the turn.
     */
    void turnFor(double angle, vex::turnType direction, int speed);

    /**
     * @brief Turns the robot a specified amount.
     * @param angle The amount of degrees to turn.
     * @param direction The direction of the turn.
     */
    void turnFor(double angle, vex::turnType direction);

    /**
     * @brief Turn the robot in an arc.
     * @param radius The radius of the arc in inches.
     * @param direction The direction to turn in.
     * @param angle The angle to turn to.
     * @param speed. Optional The top speed to turn at.
     * @warning If using a base that easily moves from side to side, it is recommended that you
     *          keep the speed less than 50. Otherwise, your robot may drift excessively.
     */
    void arcTurn(double radius, vex::turnType direction, int angle, int speed);

    /**
     * @brief Turn the robot in an arc.
     * @param radius The radius of the arc in inches.
     * @param direction The direction to turn in.
     * @param angle The angle to turn to.
     */
    void arcTurn(double radius, vex::turnType direction, int angle);

    /*----- odo tracking -----*/

    /**
     * @brief Sets the offset of the back encoder wheel and the width of the side trackers
     *
     * @param backOffset Distance of center of back wheel from robot turning center
     * @param trackWidth Distance between center of side tracking wheels
     */
    void setupOdometry(double trackWidth = 0);

    /**
     * @brief Returns the position structure that stores the odometry data
     *
     * @return Position structure with odometry data
     */
    Position returnOdoPosition();

    /**
     * @brief Stars the odometry thread.
     */
    void startOdoThread();

    /**
     * @brief Calls the function for the odometry thread.
     *!@warning DO NOT CALL THIS FUNCTION!
     */
    void odoThreadFunction();

    /************ Sensors ************/
    /*----- movement -----*/

    /**
     * @returns True if the base motors are moving.
     */
    bool isMoving();

    /**
     * @brief Gets the motor speed of the specified side.
     * @param side The side to get the speed from.
     * @returns The speed of the motors.
     */
    double getMotorSpeed(vex::turnType side);

    vex::motor* getLeftMotor(int index);
    vex::motor* getRightMotor(int index);

  private:
    /************ motors ************/

    void balanceMotors();  // checks if a motor is unplugged and disables the corrsponding motor to balance
    void motorPortsSetup(evAPI::leftAndRight side, int motorCount, int ports[]);
    void motorReverseSetup(evAPI::leftAndRight side, int motorCount, bool reverse[]);

    /*----- left motors -----*/
    void spinLeftMotors(int speed);             // spins all motors on the left side
    void stopLeftMotors(vex::brakeType type);   // stop all motors on the left side
    std::vector<vex::motor*> leftMotors;        // motors on left side of base
    std::vector<vex::motor*> activeLeftMotors;  // motors on left side of base that have not been disabled to balance

    /*----- right motors -----*/
    void spinRightMotors(int speed);             // spins all motors on the right side
    void stopRightMotors(vex::brakeType type);   // stop all motors on the right side
    std::vector<vex::motor*> rightMotors;        // motors on right side of base
    std::vector<vex::motor*> activeRightMotors;  // motors on right side of base that have not been disabled to balance

    /****** encoders ******/
    vex::rotation* leftEncoder;            // pointer to left encoder object
    vex::rotation* rightEncoder;           // pointer to right encoder object
    vex::rotation* backEncoder;            // pointer to back encoder object
    SmartEncoder* leftTracker = nullptr;   // multi tracker for left side
    SmartEncoder* rightTracker = nullptr;  // multi tracker for right Side
    SmartEncoder* backTracker = nullptr;   // multi tracker for back side

    int leftDriveTracker;   // ID for left drive tracker
    int leftOdoTracker;     // ID for left odo tracker
    int rightDriveTracker;  // ID for left drive tracker
    int rightOdoTracker;    // ID for left odo tracker
    int backOdoTracker;     // ID for back odo tracker

    double leftEncoderDegsPerInch;   // degrees per inch of wheel on left encoder
    double rightEncoderDegsPerInch;  // degrees per inch of wheel on right encoder
    double backEncoderDegsPerInch;   // degrees per inch of wheel on back encoder

    /****** inertial sensor ******/
    vex::inertial* turnSensor;  // pointer to the inertial sensor

    /****** motor and wheel settings ******/
    bool isDebugMode = false;  // is debug mode on
    int motorCount = -1;       // the number of motors on each side of drive base
    vex::gearSetting currentGear;
    float wheelSize;    // stores the diameter of wheel
    float gearInput;    // stores the input value of the gear ratio
    float gearOutput;   // stores the output value of the gear ratio
    float degsPerInch;  // store the calculated degrees per inch.

    /****** odo stuff ******/
    vex::thread* odoThread;  // thread used for odo tracking
    OdoMath* odoTracker;     // object that runs odo math

    /****** drive variables ******/
    PID turnPID;
    PID drivePID;
    PID driftPID;
    PID arcPID;
    PID arcDriftPID;
    int driveSpeed = 80;
    int turnSpeed = 60;
    int arcTurnSpeed = 40;
    double driveBaseWidth;  // distance between the two wheels from center;

    double driveP;
    double driveI;
    double driveD;
    int driveMaxStopError;  // max amount of degrees to be considered "there"
    int driveTimeToStop;    // how many pid cycles of being "there" till it stops

    double turnP;
    double turnI;
    double turnD;
    int turnMaxStopError;  // max amount of degrees to be considered "there"
    int turnTimeToStop;    // how many pid cycles of being "there" till it stops

    double driftP;
    double driftI;
    double driftD;
    int driftMaxStopError;  // max amount of degrees to be considered "there"
    int driftTimeToStop;    // how many pid cycles of being "there" till it stops

    double arcP;
    double arcI;
    double arcD;
    int arcMaxStopError;  // max amount of degrees to be considered "there"
    int arcTimeToStop;    // how many pid cycles of being "there" till it stops

    double arcDriftP;
    double arcDriftI;
    double arcDriftD;
    int arcDriftMaxStopError;  // max amount of degrees to be considered "there"
    int arcDriftTimeToStop;    // how many pid cycles of being "there" till it stops

    /****** formulas ******/
    leftAndRight findDir(int startingAngle, int endingAngle);             // finds the direction that is faster
    int turnError(leftAndRight direction, int startAngle, int endAngle);  // finds the error of a turn
};
}  // namespace evAPI

#endif  // __DRIVE_H__