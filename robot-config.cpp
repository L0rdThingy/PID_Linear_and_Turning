#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FR = motor(PORT14, ratio18_1, false);
motor MR = motor(PORT11, ratio18_1, true);
motor BR = motor(PORT13, ratio18_1, true);

motor FL = motor(PORT18, ratio18_1, true);
motor ML = motor(PORT19, ratio18_1, false);
motor BL = motor(PORT17, ratio18_1, false);

motor Ring = motor(PORT8, ratio18_1, true);
motor Lift = motor(PORT3, ratio36_1, true);

pneumatics LiftClaw = pneumatics(Brain.ThreeWirePort.H);
pneumatics Slift = pneumatics(Brain.ThreeWirePort.G);
pneumatics Dock = pneumatics(Brain.ThreeWirePort.F);

inertial IMU = inertial(PORT16);

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