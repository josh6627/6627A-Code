/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// FR                   motor         3               
// BR                   motor         4               
// FL                   motor         9               
// BL                   motor         2               
// Lift                 motor         14              
// Hook                 motor         10              
// Lift2                motor         15              
// Tilter               motor         18              
// IS                   inertial      19              
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

//FOR THE GOOGLE DOC
/*
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
*/
//FOR THE GOOGLE DOC^


// define your global instances of motors and other devices here


//Autonomous Code Global Statements

//brakes drive in either hold or coast. Ex. brake_drive();
void brake_drive() {
  FR.setStopping(hold);
  BR.setStopping(hold);
  FL.setStopping(hold);
  BL.setStopping(hold);}

void coast_drive() {
  FR.setStopping(coast);
  BR.setStopping(coast);
  FL.setStopping(coast);
  BL.setStopping(coast);}

//sets velocity using the interger vel. Ex. velocity_set(100);
void velocity_set(int vel) {
  FL.setVelocity(vel, percent);
  BL.setVelocity(vel,percent);
  FR.setVelocity(vel, percent);
  BR.setVelocity(vel,percent);}



                //These Autonomous Global Statements use the internal motor's encoders to run
        //Uses the rotateFor command, which makes the robot move for a specific amount of deg or rev
        //Uses boolean statements (true or false) to decide whether the code will run the next line immediately or wait for the line to finish


//Ex. move_drive(360, 50, true);
//move forward statement (moves in degrees)
void move_drive(int pos, int speed, bool stopping) {
  FR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  FL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
  }

//turn right statement (moves in degrees)
void turn_right(int pos, int speed, bool stopping) {
  FR.rotateFor(-pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BR.rotateFor(-pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  FL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}

//turn left statement (moves in degrees)
void turn_left(int pos, int speed, bool stopping) {
  FR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  FL.rotateFor(-pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BL.rotateFor(-pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}

//move hook statement (moves in degrees)
void move_hook(int pos, int speed, bool stopping) {
  Hook.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}

//moves tilter (moves in degrees)
void move_tilter(int pos, int speed, bool stopping) {
  Tilter.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}


//moves lift (moves in degrees) (note the first line is false because the lift uses 2 motors)
void move_lift(int pos, int speed, bool stopping) {
  Lift.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  Lift2.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);

}


//function that grabs the neutral left mogo
void grab_NLmogo(){
//moves hook and drives to nlmogo
  move_hook(720, 100, false);
  move_drive(990, 65, true);
    wait(50, msec);
  move_hook(-410, 80, false);
//nlmogo grabbed
}

//function that take the neutral left mogo back to the alliance home zone
void NLmogo_ahz(){
  move_drive(120, 20, true);
  move_drive(-600, 65, true);
    wait(100, msec);
//nlmogo brought back to ahz
}

//function that grabs the neutral middle mogo
void grab_NMmogo(){
//turn left with the mogo in the claw to align with the middle mogo
  turn_left(410, 50, true);
//let go of the mogo
  move_hook(410, 100, false);   
  move_tilter(-950, 100, false);
//move the drive reversed to the middle mogo
  move_drive(-700, 50, true);
    wait(100, msec);
  move_drive(-225, 25, true);
  turn_left(55, 50, false);
//move the tilter up with the middle mogo hooked
  move_tilter(975, 100, true);
    wait(100, msec);
}

//function that takes the neutral middle mogo to the alliance home zome
void NMmogo_ahz(){
//move the drive forward with the middle mogo
  move_drive(1250, 45, true);
}


                //reset sensors so they are more accurate
    //reset sensors once in pre auton and once in the beginning of auton
//Ex. reset_rotation();
void reset_rotation() {
  FR.resetRotation();
  BR.resetRotation();
  FL.resetRotation();
  BL.resetRotation();
  Tilter.resetRotation();
  Hook.resetRotation();
  Lift.resetRotation();
  Lift2.resetRotation();
}












//Driver Code Global Satements

                //Globals
      //Sets the constant intergers for the hook and tilter which tell the motor what degree to turn too when you click the button 
      //(See driver control code for the hook and tilter code)
      //change these by changing the number
          //NOTE: the hook starts fully withdrawn and the tilter starts fully retracted (this is when the motors will read 0 degrees and you should base the degrees off that)
const int HOOK_OUT = 700; //determines how far the hook will go to hook onto a mogo
const int HOOK_IN  = 300; //determines how far the hook will retract when not trying to hook a mogo        

const int TILTER_OUT = -1200; //determines how far the tilter will go to hook onto a mogo
const int TILTER_IN  = 10; //determines how far the tilter will retract when not trying to hook a mogo




//Motor Functions


        //Reverse Arcade Drive (the left and right joysticks are switched)
    //Ex. set_revarcade();
void set_revarcade() {
  FL.spin(forward, (Controller1.Axis2.value() + Controller1.Axis4.value()), percent);
  FR.spin(forward, (Controller1.Axis2.value() - Controller1.Axis4.value()), percent);
  BL.spin(forward, (Controller1.Axis2.value() + Controller1.Axis4.value()), percent);
  BR.spin(forward, (Controller1.Axis2.value() - Controller1.Axis4.value()), percent);
}




          //Hook Motor Functions
void set_hook_position  (int pos, int speed)  {Hook.  startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); } 
//uses pos (degrees) and speed (percent)



          //Tilter Motor Functions
void set_tilter_position  (int pos, int speed)  {Tilter.  startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); } 
//uses pos (degrees) and speed (percent)




          //Lift Motor Functions
//defines the speed of the Lift Intake and whether the Ring goes in forward or reverse
void lift() {
  Lift.spin(directionType::fwd, 65, percentUnits::pct); 
  Lift2.spin(directionType::fwd, 65,percentUnits::pct);}
//change the number to alter the speed of the lift when going up (both numbers must be the same!!!)

void liftrev() {
  Lift.spin(directionType::rev, 65, percentUnits::pct);
  Lift2.spin(directionType::rev, 65,percentUnits::pct);}
//change the number to alter the speed of the lift when going down (both numbers must be the same!!!)

void liftbrake() {
  Lift.stop(brakeType::hold);
  Lift2.stop(brakeType::hold);}
//Lift brake types (hold,coast,brake)










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
  reset_rotation();
    wait(200, msec);
  
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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  reset_rotation();

//first number is degrees and second is the speed

  grab_NLmogo();
  NLmogo_ahz();
  grab_NMmogo();
  NMmogo_ahz();


  coast_drive();
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

bool hook_up  =true;
int hook_lock      = 0;

bool tilter_up  =false;
int tilter_lock      = 0;


  // User control code here, inside the loop
  while (1) {

//sets drive to coast
  coast_drive();


//main reverse arcade drive code
  set_revarcade();







//Lift code
//refers to lift global statements
  if(Controller1.ButtonL1.pressing()){
    lift();}

  else if(Controller1.ButtonL2.pressing()){
    liftrev();}

  else{
    liftbrake();}







//Hook
//two positions up and perfect grab for locking onto mogos
  if (Controller1.ButtonR1.pressing() && hook_lock==0) {
    hook_up = !hook_up;
    hook_lock = 1;}

  else if (!Controller1.ButtonR1.pressing()) {
    hook_lock = 0;}


  if (hook_up)
    set_hook_position(HOOK_OUT, 100); //sets speed of the hook when going out

  else
    set_hook_position(HOOK_IN, 100); //sets speed of the hook when going back in







//Tilter Code
//two positions up and perfect grab for locking onto mogos
  if (Controller1.ButtonR2.pressing() && tilter_lock==0) {
    tilter_up = !tilter_up;
    tilter_lock = 1;}

  else if (!Controller1.ButtonR2.pressing()) {
    tilter_lock = 0;}


  if (tilter_up)
    set_tilter_position(TILTER_OUT, 100); //sets speed of the tilter when going out

  else
    set_tilter_position(TILTER_IN, 100); //sets speed of the tilter when going back in







}
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................








    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
