#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FR = motor(PORT3, ratio18_1, true);
motor BR = motor(PORT4, ratio18_1, true);
motor FL = motor(PORT9, ratio18_1, false);
motor BL = motor(PORT2, ratio18_1, false);
motor Lift = motor(PORT14, ratio36_1, false);
motor Hook = motor(PORT10, ratio18_1, false);
motor Lift2 = motor(PORT15, ratio36_1, true);
motor Tilter = motor(PORT18, ratio18_1, false);
inertial IS = inertial(PORT19);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}