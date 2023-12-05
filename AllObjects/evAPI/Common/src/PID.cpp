/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       PID.cpp                                                   */
/*    Author:       Jackson Area Robotics                                     */
/*    Created:      ???                                                       */
/*    Description:  PID class.                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include <math.h>
#include "../../Common/include/PID.h"

namespace evAPI
{
  PID::PID(double error, double kp, double ki, double kd, double starti) :
    //error(error),
    KP(kp),
    KI(ki),
    KD(kd),
    starti(starti)
  {};

  PID::PID(double error, double kp, double ki, double kd, double starti, double settleError, double settleTime, double timeout) :
    //error(error),
    KP(kp),
    KI(ki),
    KD(kd),
    starti(starti),
    settleError(settleError),
    settleTime(settleTime),
    timeout(timeout)
  {};

  double PID::compute(double error){
    if (fabs(error) < starti){
      accumulatedError+=error;
    }
    if ((error>0 && previousError<0)||(error<0 && previousError>0)){ 
      accumulatedError = 0; 
    }
    output = KP*error + KI*accumulatedError + KD*(error-previousError);

    previousError=error;

    if(fabs(error)<settleError){
      timeSpentSettled+=10;
    } else {
      timeSpentSettled = 0;
    }

    timeSpentRunning+=10;

    return output;
  }

  bool PID::isSettled()
  {
    if((timeSpentRunning > timeout && timeout != 0) || (timeSpentSettled > settleTime))
    { return(true); }
    
    return(false);
  }

  void PID::setConstants(double kp, double ki, double kd)
  {
    KP = kp;
    KI = ki;
    KD = kd;
  }
}