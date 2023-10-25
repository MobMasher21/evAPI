/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Cameron Barclay                                           */
/*    Created:      10/11/2023, 10:30:35 PM                                   */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"
#include "visionConfig.h"
#include "../evAPI/evAPIFiles.h"

using namespace vex;

// define your global instances of motors and other devices here

VisionTracker objectTracker(&VisionSensor, &OBJECT_COLOR);


int main() {

    //vision tracking settings
    objectTracker.setLockRange(20);
    objectTracker.tglDebugMode(true);
    objectTracker.tglLEDIndicator(true);
    objectTracker.tglScreenIndicator(true);
    objectTracker.tglDistanceCalculation(true);

    Brain.Screen.printAt( 10, 50, "Hello V5" );
   
    while(1) {
        
        // Allow other tasks to run
        this_thread::sleep_for(10);
    }
}
