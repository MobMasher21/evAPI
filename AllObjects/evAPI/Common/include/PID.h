/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PID.h                                                     */
/*    Author:       Jackson Area Robotics                                     */
/*    Created:      ???                                                       */
/*    Description:  PID class.                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#ifndef PID_H
#define PID_H

namespace evAPI
{
  class PID
  {
    private:
      //double error = 0;
      double KP = 0;
      double KI = 0;
      double KD = 0;
      double starti = 0;
      double settleError = 0;
      double settleTime = 0;
      double timeout = 0;
      long accumulatedError = 0;
      double previousError = 0;
      double output = 0;
      double timeSpentSettled = 0;
      double timeSpentRunning = 0;

    public:

      /**
       * @brief Creates a PID object.
       * @param error Starting error
       * @param kp KP Value
       * @param ki KI Value
       * @param kd KD Value
       * @param starti
       * @param settleError
       * @param settleTime
       * @param timeout
      */
      PID(double error, double kp, double ki, double kd, double starti, double settleError, double settleTime, double timeout);

      PID(double error, double kp, double ki, double kd, double starti);

      double compute(double error);

      bool isSettled();

      /**
       * @brief Sets constants used for the PID function.
       * @param kp The KP factor.
       * @param ki The KI factor.
       * @param kd The KD factor.
      */
      void setConstants(double kp, double ki, double kd);
  };
}

#endif // PID_H