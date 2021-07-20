#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor lb = motor(PORT3, ratio18_1, true);
motor lf = motor(PORT11, ratio18_1, true);
motor rb = motor(PORT8, ratio18_1, false);
motor rf = motor(PORT9, ratio18_1, false);
motor conveyor = motor(PORT1, ratio18_1, true);
motor intake = motor(PORT6, ratio18_1, true);
motor mogo = motor(PORT18, ratio18_1, false);
motor tilter = motor(PORT17, ratio36_1, true);

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