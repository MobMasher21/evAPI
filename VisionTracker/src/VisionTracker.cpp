#include "../include/VisionTracker.h"

VisionTracker::VisionTracker(vex::vision* sensorAddress, vex::vision::signature* signatureAddress) {
  fflush(stdout);
  sensorPointer = sensorAddress;
  trackingSignature = signatureAddress;
}

bool VisionTracker::tglLEDIndicator(bool state) {
  LEDMode = state;
  return LEDMode;
}

bool VisionTracker::tglScreenIndicator(bool state) {
  screenMode = state;
  return screenMode;
}

bool VisionTracker::tglDebugMode(bool state) {
  debugMode = state;
  return debugMode;
}

bool VisionTracker::tglDistanceCalculation(bool state) {
  distanceMode = state;
  for (int i = 0; i < AVERAGE_COUNT; i++) {
    averageNumbers[i] = 0;
  }
  return distanceMode;
}

void VisionTracker::setupDistanceMath(double a, double b, double c) {
  aValue = a;
  bValue = b;
  cValue = c;
}

int VisionTracker::setLockRange(int range) {
  lockRange = range;
  lockTop = 157 + range;  // 157 is half of the total vision sensor range
  lockBottom = 157 - range;
  return lockRange;
}

double VisionTracker::returnAverageDistance() {
  return (averageSum / AVERAGE_COUNT);
}

void VisionTracker::trackObject() {
  sensorPointer->takeSnapshot(*trackingSignature);
  // sets variables to biggest object values
  objectXDim = sensorPointer->objects[0].width;
  objectYDim = sensorPointer->objects[0].height;
  objectXPos = sensorPointer->objects[0].centerX;
  objectYPos = sensorPointer->objects[0].centerY;
  objectCount = sensorPointer->objectCount;

  if (objectCount > 0) {
    // calculates if it is locked and which direction off it is
    if (objectXPos >= lockBottom && objectXPos <= lockTop) {
      isLocked = true;
    } else {
      isLocked = false;
      if (objectXPos > lockTop) {
        directionOff = evAPI::LEFT;
      } else if (objectXPos < lockBottom) {
        directionOff = evAPI::RIGHT;
      }
    }
  }

  // run distance summations
  if (objectCount > 0) {
    if (distanceMode) {
      averageSum = averageSum - averageNumbers[tailPosition];
      tailPosition++;
      headPosition++;
      if (tailPosition == AVERAGE_COUNT) tailPosition = 0;
      if (headPosition == AVERAGE_COUNT) headPosition = 0;
      averageNumbers[headPosition] = findDistance(objectXDim);
      averageSum = averageSum + averageNumbers[headPosition];
    }
  }

  // run calibration summations
  if (objectCount > 0) {
    if (calibrationMode) {
      averageSum = averageSum - averageNumbers[tailPosition];
      tailPosition++;
      headPosition++;
      if (tailPosition == AVERAGE_COUNT) tailPosition = 0;
      if (headPosition == AVERAGE_COUNT) headPosition = 0;
      averageNumbers[headPosition] = objectXDim;
      averageSum = averageSum + averageNumbers[headPosition];
    }
  }

  // prints object to screen
  if (screenMode) {
    Brain.Screen.clearScreen();
    if (objectCount > 0) {
      // draw border
      Brain.Screen.setFillColor(vex::color::black);
      Brain.Screen.setPenColor(vex::color::white);
      Brain.Screen.setPenWidth(5);
      Brain.Screen.drawRectangle(10, 10, 326, 222);

      // set draw object
      Brain.Screen.setPenWidth(1);
      if (isLocked) {
        Brain.Screen.setFillColor(vex::color::blue);
      } else {
        if (directionOff == evAPI::LEFT) {
          Brain.Screen.setFillColor(vex::color::green);
        } else if (directionOff == evAPI::RIGHT) {
          Brain.Screen.setFillColor(vex::color::red);
        }
      }
      Brain.Screen.drawRectangle(sensorPointer->objects[0].originX + 10, sensorPointer->objects[0].originY + 10, objectXDim, objectYDim);
    }
  }

  // set sensor LED color
  if (LEDMode) {
    if (objectCount > 0) {
      if (isLocked) {
        sensorPointer->setLedColor(0, 0, 255);
      } else {
        if (directionOff == evAPI::LEFT) {
          sensorPointer->setLedColor(0, 255, 0);
        } else if (directionOff == evAPI::RIGHT) {
          sensorPointer->setLedColor(255, 0, 0);
        }
      }
    } else {
      sensorPointer->setLedColor(0, 0, 0);
    }
  }

  // print data to terminal
  if (debugMode) {
    if (distanceMode) {
      printf("ball distance = %f\n", returnAverageDistance());
      printf("averageSum = %f, ", averageSum);
      printf("headPos = %i, ", headPosition);
      printf("tailPos = %i\n", tailPosition);
    }
    printf("xPos = %i, ", objectXPos);
    printf("yPos = %i, ", objectYPos);
    printf("xDim = %i, ", objectXDim);
    printf("yDim = %i, ", objectYDim);
    printf("objCnt = %i, ", objectCount);
    printf("isLock = %i\n\n", isLocked);
  }
  if (calibrationMode) printf("%i\n\n", int(returnAverageDistance()));
}

void VisionTracker::enableCalibration() {
  tglDebugMode(false);
  tglDistanceCalculation(false);
  calibrationMode = true;
}

int VisionTracker::getObjectCount() {
  return (objectCount);
}

int VisionTracker::getObjectXDim() {
  return (objectXDim);
}

int VisionTracker::getObjectYDim() {
  return (objectYDim);
}

int VisionTracker::getObjectXPos() {
  return (objectXPos);
}

int VisionTracker::getObjectYPos() {
  return (objectYPos);
}

bool VisionTracker::getLockedState() {
  return (isLocked);
}

evAPI::leftAndRight VisionTracker::getOffDirection() {
  return (directionOff);
}