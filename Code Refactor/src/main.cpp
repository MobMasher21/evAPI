// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// triballSensor        distance      8               
// ArmDown              digital_out   G               
// ArmUp                digital_out   H               
// Inertial             inertial      17              
// ---- END VEXCODE CONFIGURED DEVICES ----

/** code ideas
* write automatic controller debouncing code as a class
*/

#include "vex.h"
#include "evAPI.h"

using namespace evAPI;

Drive driveBase; //Setup code objects
competition Competition;
goodUI UI;

int batteryCapacity = 0; //Variables that store info about the battery
double batteryVolt = 0;
double batteryCurrent = 0;
double batteryTemp = 0;

int UIIndex = UI.getProgNumber() + 1;

//Controls for the robot arm.
void TriballArmDown()
{
  ArmUp.set(false);
  ArmDown.set(true);
}

void TriballArmUp()
{
  ArmUp.set(true);
  ArmDown.set(false);
}

/* int leftSpeed;
int rightSpeed; */

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  //Setup Base
  driveBase.setGearbox(blueGearBox);
  driveBase.leftPortSetup(19, 10);
  driveBase.rightPortSetup(9, 20);
  driveBase.leftReverseSetup(true, true);
  driveBase.rightReverseSetup(false, false);
  driveBase.setGeneralHandicap(0.9);
  driveBase.setTurningHandicap(0.6);

  /* driveBase.leftEncoderSetup(1, 2.75, false);
  driveBase.rightEncoderSetup(2, 2.75, false);
  driveBase.backEncoderSetup(6, 2.75, false); */

  //Setup Preauto UI
  UI.setDebugMode(false);

  UI.addButton(0, blue);
  UI.addIcon(0, UI.Icons.exclamationMark);
  UI.addTitle(0, "Test Title!");
  UI.addDescription(0, "Why do I exist? :(");

  UI.addButton(1, red);
  UI.addIcon(1, UI.Icons.number1);

  UI.addButton(2, noAlliance);
  UI.addIcon(2, UI.Icons.snow);

  UI.addBlank();
  UI.addBlank();
  UI.addBlank();
  UI.addBlank();
  UI.addBlank();


  UI.addButton(8, vexClrChocolate);
  UI.setDisplayTime(2000);

  //Setup Match UI
  UI.setDefaultReadOutColor(blue);
  UI.createBrainReadOut("Battery: ", batteryCapacity);
  UI.createBrainReadOut("Battery Volt: ", batteryVolt);
  UI.createBrainReadOut("Battery Amps: ", batteryCurrent);
  UI.createBrainReadOut("Bat Degrees F: ", batteryTemp, red);

  //Setup Controller UI
  UI.createControllerReadOut("Battery: ", batteryCapacity);
  UI.createControllerReadOut("Auto: ", UIIndex);
  UI.createControllerReadOut("Refactor");

  //Start UI
  Inertial.calibrate();

  Inertial.setRotation(5, degrees);

  while(Inertial.isCalibrating())
  { this_thread::sleep_for(10); }

  UI.selectButton(0, true);
  UI.startUIThreads();
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void usercontrol(void) {

    //=================================================================================
    vex::task::sleep(20); // Sleep the task for a short amount of time to
                          // prevent wasted resources.
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  //! Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  TriballArmUp();

  // Run the pre-autonomous function.
  pre_auton();

  while (true) {
    batteryCapacity = Brain.Battery.capacity();
    batteryVolt = Brain.Battery.voltage(volt);
    batteryCurrent = Brain.Battery.current(amp);
    batteryTemp = Brain.Battery.temperature(temperatureUnits::fahrenheit);
    UIIndex = UI.getProgNumber() + 1;

    this_thread::sleep_for(10);
  }
}