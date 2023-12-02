/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       Cameron Barclay                                           */
/*    Created:      9/30/2023, 12:45:32 AM                                    */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

#include "vex.h"
#include "../evAPI/evAPIFiles.h"

using namespace vex;

AutoSelector autoSelector;

int main() {
  autoSelector.addButton(0, red);
  autoSelector.addButton(1, blue);
  autoSelector.addButton(2, green);
  autoSelector.addButton(3, yellow);
  autoSelector.setButtonIcon(0, autoSelector.icons.number5);
  autoSelector.setButtonIcon(1, autoSelector.icons.number7);
  autoSelector.setButtonIcon(2, autoSelector.icons.number4);
  autoSelector.setButtonIcon(3, autoSelector.icons.number9);
  autoSelector.setButtonTitle(0, "Hello");
  autoSelector.setButtonTitle(1, "bye");
  autoSelector.setButtonDescription(1, "This is a very very very very very very very very very very very long sentence just to test things out\noh and there was also a new line");
  autoSelector.printButtons();
  autoSelector.startThread();
  while (true) {
    wait(100, msec);
  }
}
