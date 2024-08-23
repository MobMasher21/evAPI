#ifndef SMARTENCODER_H_
#define SMARTENCODER_H_

#include <vector>

#include "../../../Common/include/generalFunctions.h"

class SmartEncoder {
  public:
    SmartEncoder(vex::motor* motorEncoderObject);                   // constructor to set motor pointer
    SmartEncoder(vex::rotation* rotationEncoderObject);             // constructor to set rotation sensor pointer
    void setEncoderMotor(vex::motor* motorEncoderObject);           // accessor to change motor encoder
    void setEncoderRotation(vex::rotation* rotationEncoderObject);  // accessor to set rotation sensor
    void resetAll();                                                // resets raw encoder and all offsets
    int newTracker();                                               // adds a new septate tracker
    void resetTrackerPosition(int trackerID);                       // resets a specified tracker
    double readTrackerPosition(int trackerID);                      // reads a tracker of a specified tracker

  private:
    std::vector<double> encoderOffsets;        // stores all the offsets with to an array
    int trackerCount;                          // how many encoders are being used
    vex::motor* motorEncoder = nullptr;        // pointer to vex motor
    vex::rotation* rotationEncoder = nullptr;  // pointer to vex rotation sensor
    double encoderRead();                      // reads the raw encoder, motor or rotation
    void encoderReset();                       // resets the encoder, motor or rotation
};

#endif  // SMARTENCODER_H_