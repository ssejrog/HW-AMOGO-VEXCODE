using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor lb;
extern motor lf;
extern motor rb;
extern motor rf;
extern motor conveyor;
extern motor intake;
extern motor mogo;
extern motor tilter;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );