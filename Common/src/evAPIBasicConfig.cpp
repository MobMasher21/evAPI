#include "../include/evAPIBasicConfig.h"

vex::brain Brain;
vex::controller primaryController = vex::controller(vex::controllerType::primary);
vex::controller secondaryController = vex::controller(vex::controllerType::partner);
vex::competition Competition = vex::competition();