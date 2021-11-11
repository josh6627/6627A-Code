#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FR = motor(PORT11, ratio18_1, false);
motor BR = motor(PORT7, ratio18_1, false);
motor FL = motor(PORT13, ratio18_1, true);
motor BL = motor(PORT14, ratio18_1, true);
motor Mogo = motor(PORT15, ratio36_1, false);
motor Ring = motor(PORT5, ratio18_1, true);
motor LeftLift = motor(PORT6, ratio36_1, true);
motor RightLift = motor(PORT8, ratio36_1, false);
pneumatics Claw = pneumatics(Brain.ThreeWirePort.H);
inertial IMU = inertial(PORT10);

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