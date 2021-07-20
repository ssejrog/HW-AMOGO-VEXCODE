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
// lb                   motor         3               
// lf                   motor         11              
// rb                   motor         8               
// rf                   motor         9               
// conveyor             motor         1               
// intake               motor         6               
// mogo                 motor         18              
// tilter               motor         17              
// ---- END VEXCODE CONFIGURED DEVICES ----


#include "vex.h"
using namespace vex;


///
// Globals
//  - it's good practice to name globals in all caps
//  - we also try to avoid "hard coding" number.  use variables wherever
//  - you can, so you can change things when you eventually need to
///
int         mogo_rev   =  1;             // If the mobile goal lift needs to be reversed, DO NOT REVERSE THE MOTOR, flip this variable instead!
const int   MOGO_OUT   =  1150*mogo_rev; // Out position for the mogo lift
const int   MOGO_NEUT  =  700*mogo_rev;  // Neutral position

const int  RESET_TIMEOUT     = 3500; // Time for resetting sensors to give up
bool       WAS_RESET_SUCCESS = true; // Flag for if resetting worked

const float SCALE      =  12000/100; // Makes input out of 100 instead of 12000
const int   THRESH     =  5;         // When joystick is within this, the motors will set to 0.  This is the deadzone
const int   DELAY_TIME =  10;        // Delay time for loops (ms)

const int TILTER_IN  = -20;
const int TILTER_OUT = -315;



///
// Set Motor Functions
//  - this sets motors between -12000 and 12000.  i'm used to
//  - -100 to 100, so the "scale" variable lets me use that as
//  - inputs and scales it to -12000 to 12000
///

// Set voltage
void set_tank(int l, int r) {
  lf.spin(fwd, l*SCALE, voltageUnits::mV);
  lb.spin(fwd, l*SCALE, voltageUnits::mV);
  rb.spin(fwd, r*SCALE, voltageUnits::mV);
  rf.spin(fwd, r*SCALE, voltageUnits::mV);
}
void set_mogo    (int input) { mogo.    spin(fwd, input*SCALE*mogo_rev, voltageUnits::mV); }
void set_conveyor(int input) { conveyor.spin(fwd, input*SCALE,          voltageUnits::mV); }
void set_intake  (int input) { intake.  spin(fwd, input*SCALE,          voltageUnits::mV); }
void set_tilter  (int input) { tilter.  spin(fwd, input*SCALE,          voltageUnits::mV); }

// Set position
void set_mogo_position   (int pos, int speed) { mogo.    startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); }
void set_tilter_position (int pos, int speed) { tilter.  startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); }

int get_mogo() { return mogo.rotation(deg)*mogo_rev; }

// Brake modes
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
        WAS_RESET_SUCCESS = false;
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
}



///
// Main
///
int main() {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  zero_sensors();

  // If zeroing was not successful, print to screen
  if (!WAS_RESET_SUCCESS) {
    Controller1.rumble("-"); // Vibrate the controller so the user knows something is wrong

    Brain.Screen.setPenColor(red);
    Brain.Screen.setFillColor(red);
    Brain.Screen.drawRectangle(0, 0, 480, 272);

    for (int i=0;i<15;i++) {
      Brain.Screen.setPenColor(white);
      Brain.Screen.printAt(100, 16*(i+1), "!SUB-SYSTEMS DID NOT ZERO!");
      wait(DELAY_TIME, msec);
    }
  } 
  // If zeroing was successful, make the brain green
  else {
    Brain.Screen.setPenColor(green);
    Brain.Screen.setFillColor(green);
    Brain.Screen.drawRectangle(0, 0, 480, 272);
  }

  // Parameters for user control
  bool mogo_up;
  bool neut = 0;
  int mogo_lock = 0;
  bool is_up = true;
  int mogo_timer = 0;

  bool tilter_up;
  bool tilter_lock = 0;

  mogo_up = true;
  tilter_up = true;

  int intake_conveyor_speed = 0;

  while (true) {

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
    //  - when L1 is held, the mogo lift will go to the neutral position
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
        set_mogo(-100);
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
        set_mogo(100);
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
      intake_conveyor_speed =  100;
    }
    else if (Controller1.ButtonR2.pressing()) {
      intake_conveyor_speed = -100;
    }
    else {
      intake_conveyor_speed =  0;
    }
    set_intake  (intake_conveyor_speed);
    set_conveyor(intake_conveyor_speed);



    wait(DELAY_TIME, msec); // Don't hog the CPU!
  }
}
