/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       C:\Users\Jess                                             */
/*    Created:      Thu Jul 15 2021                                           */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// lb                   motor         13              
// lf                   motor         11              
// rb                   motor         14              
// rf                   motor         21              
// conveyor             motor         12              
// intake               motor         17              
// mogo                 motor         18              
// tilter               motor         15                  
// ---- END VEXCODE CONFIGURED DEVICES ----



#include "vex.h"
using namespace vex;

// A global instance of competition
competition Competition;



///
// Globals
//  - it's good practice to name globals in all caps
//  - we also try to avoid "hard coding" number.  use variables wherever
//  - you can, so you can change things when you eventually need to
///
const int RESET_TIMEOUT     = 3500;  // Time for resetting sensors to give up
bool      WAS_RESET_SUCCESS = false; // Flag for if resetting worked

const int THRESH     =  5;  // When joystick is within this, the motors will set to 0.  This is the deadzone
const int DELAY_TIME =  10; // Delay time for loops (ms)

const int MOGO_OUT   =  1150; // Out position for the mogo lift
const int MOGO_NEUT  = 700;   // Neutral Mogo position

const int TILTER_IN  = -20;  // Tilter In Position
const int TILTER_OUT = -315; // Tilter Out Position



///
// Set Motor Functions
//  - this sets motors between -12000 and 12000.  i'm used to
//  - -100 to 100, so the "scale" variable lets me use that as
//  - inputs and scales it to -12000 to 12000
///

// Set voltage
const int SCALE = 120;
void set_tank(int l, int r) {
  lf.spin(fwd, l*SCALE, voltageUnits::mV);
  lb.spin(fwd, l*SCALE, voltageUnits::mV);
  rb.spin(fwd, r*SCALE, voltageUnits::mV);
  rf.spin(fwd, r*SCALE, voltageUnits::mV);
}

// Drive Brake Types
void brake_drive() {
  lf.setStopping(hold);
  lb.setStopping(hold);
  rf.setStopping(hold);
  rb.setStopping(hold);
}
void coast_drive() {
  lf.setStopping(coast);
  lb.setStopping(coast);
  rf.setStopping(coast);
  rb.setStopping(coast);
}

void set_mogo    (int input) { mogo.    spin(fwd, input*SCALE, voltageUnits::mV); }
void set_conveyor(int input) { conveyor.spin(fwd, input*SCALE, voltageUnits::mV); }
void set_intake  (int input) { intake.  spin(fwd, input*SCALE, voltageUnits::mV); }
void set_tilter  (int input) { tilter.  spin(fwd, input*SCALE, voltageUnits::mV); }

// Set position
void set_mogo_position   (int pos, int speed) { mogo.    startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); }
void set_tilter_position (int pos, int speed) { tilter.  startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); }


int get_mogo() {
  return mogo.rotation(deg);
}



///
// Reset Sensors
//  - we need all mechanisms start in a consistent place so we can move relatively off of
//  - the starting position.  this function moves all mechanisms into their hardstops, and 
//  - stops once the velocity of the mechanism is 0 (the mechanism has hit the hardstop)
///
void zero_sensors() {
  bool run = true;
  bool mogo_zero   = false;
  bool tilter_zero = false;

  bool last_mogo   = false;
  bool last_tilter = false;

  int timeout_timer = 0;

  set_mogo(-40);
  set_tilter(40);

  wait(200, msec);
  while (run) {
    last_mogo   = mogo_zero;

    // Bring mogo back until velocity is 0
    if (mogo.velocity(percentUnits::pct) == 0) {
      set_mogo(0);
      mogo_zero = true;
      mogo.resetPosition();
    }

    // Bring Tilter back until velocity is 0
    if (tilter.velocity(percentUnits::pct) == 0) {
      set_tilter(0);
      tilter_zero = true;
      tilter.resetPosition();
    }

    // Timeout 
    if (mogo_zero && tilter_zero) {
      run = false;
    }

    // Check if any of the subsystems have reset, and if they have, reset timeout_timer to 0
    if (last_mogo!=mogo_zero || last_tilter!=tilter_zero) {
      timeout_timer = 0;
    } else {
      // Once timeout_timer is greater then RESET_TIMER, exit this loop
      timeout_timer+=DELAY_TIME;
      if (timeout_timer>RESET_TIMEOUT) {
        run = false;
      }
    }

    wait(DELAY_TIME, msec); // Don't hog the CPU!
  }
  // Make sure no motors are running coming out of this loop
  set_mogo  (0);
  set_tilter(0);

  // If a mechanism didn't zero, reset it here
  if (!mogo_zero)   mogo.  resetPosition();
  if (!tilter_zero) tilter.resetPosition();
  WAS_RESET_SUCCESS = true;
}



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
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  zero_sensors(); 
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
void turn_90(int dir) {
  set_tank(80*dir, -80*dir);
  wait(400, msec);
  set_tank(0, 0);
}

bool did_auto_finish = false;
void autonomous(void) {
  brake_drive();

  int exit = 0;
  while (exit<1000) {
    exit+=DELAY_TIME;
    if (abs(get_mogo())>abs(MOGO_OUT-100)) {
      if (mogo.velocity(pct)==0) 
        set_mogo(0);
      else 
        set_mogo(20);
     }
     else {
      set_mogo(127);
     }
     wait(DELAY_TIME, msec);
  }
  exit=0;
  set_tilter_position(TILTER_OUT, 100);
  
  set_tank(-127, -127);
  wait(500, msec);
  set_tank(0, 0);
  wait(500, msec);

  bool is_upp = false;
  while (exit<1000) {
    exit+=DELAY_TIME;
      if (abs(get_mogo())<150) {
        if (mogo.velocity(pct)==0) {
          is_upp = true;
          set_mogo(0);
        }
        else {
          set_mogo(is_upp?0:-20);
        }
      }
      else {
        is_upp = false;
        set_mogo(-127);
      }
    wait(DELAY_TIME, msec);
  } 
  exit=0;
  is_upp = true; 

  set_conveyor(127);
  set_intake(127);
  wait(1500, msec);
  set_conveyor(0);
  set_intake(0);

  turn_90(-1);
  wait(300, msec);

  set_tank(-127,-127);
  wait(400, msec);

  turn_90(1);
  wait(300, msec);


  set_tank(80, 80);
  wait(500, msec);
  set_tank(0, 0);

  while (exit<1000) {
    exit+=DELAY_TIME;
    if (abs(get_mogo())>abs(MOGO_OUT-100)) {
      if (mogo.velocity(pct)==0) 
        set_mogo(0);
      else 
        set_mogo(20);
     }
     else {
      set_mogo(127);
     }
     wait(DELAY_TIME, msec);
  }
  did_auto_finish = true;
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
  // User control code here, inside the loop
  // Parameters for user control
  bool mogo_up;
  bool neut = 0;
  int mogo_lock = 0;
  bool is_up = true;
  int mogo_timer = 0;

  bool tilter_up;
  bool tilter_lock = 0;

  if (did_auto_finish) {
    mogo_up = false;
    tilter_up = false;
  }

  int intake_conveyor_speed = 0;

  //checks if pre_auton ran and if did not run pre_auto
  while(WAS_RESET_SUCCESS == false){
    wait(10, msec);
  }

  coast_drive();

  while (1) {
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    
    ///
    // Joysticks
    ///

    // Ternary operator (one line if statement) for controller deadzone
    // int x = some condition   ?   x is this if condition was true   :   x is this if condition was false;
    int l_joy = abs(Controller1.Axis3.value())>THRESH ? Controller1.Axis3.value() : 0;
    int r_joy = abs(Controller1.Axis2.value())>THRESH ? Controller1.Axis2.value() : 0;
    set_tank(l_joy, r_joy);
    


    ///
    // Mogo
    //  - mogo has two positions, pressing L1 toggles it between them
    ///

    // Flip boolean when button is pressed
    if (Controller1.ButtonL1.pressing() && mogo_lock==0) {
      if (neut) {
        mogo_up = false;
      } else {
        mogo_up = !mogo_up;
      }
      neut = false;
      mogo_lock = 1;
      // mogo_lock makes it so this if statement only runs once when the button is pressed
      // if this lock didn't exist, while the button is pressed, mogo_up would spam between
      // true and false, causing the mechanism to jitter
    } 
    else if (Controller1.ButtonL1.pressing()) {
      if (mogo_up) {
        mogo_timer+=DELAY_TIME;
        if (mogo_timer>=300) {
          neut = true;
        }
      }

    }
    else if (!Controller1.ButtonL1.pressing()) {
      mogo_lock = 0;
      mogo_timer = 0;
    }

    // Have the motor go to a position depending on boolean state.
    // This runs the motor at full power until the velocity of the motor is 0.
    // when the velocity is 0, we know the subsystem has reached a hardstop.
    // then it sets the motor to a low amount of power to stop it from overheating
    if (neut) {
      set_mogo_position(MOGO_NEUT, 100);
    }
    else if (mogo_up) {
      if (abs(get_mogo())<150) {
        if (mogo.velocity(pct)==0) {
          is_up = true;
          set_mogo(0);
        }
        else {
          set_mogo(is_up?0:-20);
        }
      }
      else {
        is_up = false;
        set_mogo(-127);
      }
    }
    else if (!mogo_up) {
      if (abs(get_mogo())>abs(MOGO_OUT-100)) {
        if (mogo.velocity(pct)==0) 
          set_mogo(0);
        else 
          set_mogo(20);
      }
      else {
        set_mogo(127);
      }
    }



    ///
    // Tilter
    //  - tilter has two positions, pressing L2 toggles it between them
    ///
    // Flip boolean when button is pressed
    if (Controller1.ButtonL2.pressing() && tilter_lock==0) {
      tilter_up = !tilter_up;
      tilter_lock = 1;
      // tilter_lock makes it so this if statement only runs once when the button is pressed
      // if this lock didn't exist, while the button is pressed, tilter_up would spam between
      // true and false, causing the mechanism to jitter
    } 
    else if (!Controller1.ButtonL2.pressing()) {
      tilter_lock = 0;
    }

    // Have the motor go to a position depending on boolean state
    if (tilter_up) 
      set_tilter_position(TILTER_IN, 100);
    else 
      set_tilter_position(TILTER_OUT, 100);



    ///
    // Intake and Conveyor
    ///
    if (Controller1.ButtonR1.pressing()) {
      intake_conveyor_speed =  127;
    }
    else if (Controller1.ButtonR2.pressing()) {
      intake_conveyor_speed = -127;
    }
    else {
      intake_conveyor_speed =  0;
    }
    set_intake  (intake_conveyor_speed);
    set_conveyor(intake_conveyor_speed);

    wait(DELAY_TIME, msec); // Don't hog the CPU!
  }
}



//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}