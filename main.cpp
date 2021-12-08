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
// FR                   motor         11              
// BR                   motor         7               
// FL                   motor         13              
// BL                   motor         14              
// Mogo                 motor         12              
// Ring                 motor         5               
// LeftLift             motor         10              
// RightLift            motor         8               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath> // allows (std::abs)

using namespace vex;

// A global instance of competition
competition Competition;






///////////////////////
//    Brake Types   //
/////////////////////


//brakes drive in either hold or coast
void set_hold() {
  FR.setStopping(hold); BR.setStopping(hold); FL.setStopping(hold); BL.setStopping(hold); }

void set_coast() {
  FR.setStopping(coast); BR.setStopping(coast); FL.setStopping(coast); BL.setStopping(coast); }

void coast_drive(){
FR.stop(coast); BR.stop(coast); FL.stop(coast); BL.stop(coast); }

void brake_drive(){
FR.stop(hold); BR.stop(hold); FL.stop(hold); BL.stop(hold); }

void reset_rotation() {
  FR.resetRotation(); BR.resetRotation(); FL.resetRotation(); BL.resetRotation(); Claw.close(); LeftLift.resetRotation(); RightLift.resetRotation(); }

void set_position(int pos){
  FR.setPosition(pos, deg); BR.setPosition(pos, deg); FL.setPosition(pos, deg); BL.setPosition(pos, deg); }












     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //    Driver Control Global Statements    //
 //                                        //
//                                        //
///////////////////////////////////////////



////////////////////////////
//    Mogo Statements    //
//////////////////////////

const int MOGO_OUT = 280;
const int MOGO_IN = 0;

          //Mogo Motor Functions
void set_mogo_position  (int pos, int speed)  {Mogo.  startRotateTo(pos, rotationUnits::deg, speed, velocityUnits::pct); } 
//uses pos (degrees) and speed (percent)






////////////////////////////
//    Ring Statements    //
//////////////////////////

void ring() { //85.416667  .91666667
  Ring.spin(directionType::fwd, 11, voltageUnits::volt);}
//change the number to alter the speed of the ring intake when moving forward

void ringRev() {
  Ring.spin(directionType::rev, 11, voltageUnits::volt);}
//change the number to alter the speed of the ring intake when reversed

void ringBrake() {
  Ring.stop(brakeType::coast);}
//Ring brake types (Uses coast but could use hold too)












     ////////////////////////////////////////
    //                                    //
   //                                    //
  //    Autonomous Global Statements    //
 //                                    //
//                                    //
///////////////////////////////////////
void move_drive(int pos, int speed, bool stopping) {
  FR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  FL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}
// sets the drive motors spin for a certain amount of speeed (defined with 'pos'
// sets a certain speed (defined with 'speed')
// With 'stopping' determines whether you want the statement to be blocking or not (next line of code will run if set to false)



//moves mogo (moves in degrees)
void move_mogo(int pos, int speed, bool stopping) {
  Mogo.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}
// same logic as move_drive but with the ring motor


//rotates the ring intake
void move_ring(int pos, int speed, bool stopping) {
  Ring.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}

//ditto












     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //        Pre-Autonomous Functions        //
 //                                        //
//                                        //
///////////////////////////////////////////

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

    IMU.calibrate(); //calibrates the IMU sensor to reset the rotation
    reset_rotation(); //resets all motors
      wait(2250, msec);

}







     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //             Autonomous Task            //
 //                                        //
//                                        //
///////////////////////////////////////////

void autonomous(void) {
    reset_rotation(); // reset motors again (failsafe)


  
    Claw.open();
  move_ring(100, 50, false);
move_drive(-950, 65, true);
    turn_left(15, 40, true);
  move_drive(-125, 30, true);
  move_drive(550, 40, false);
    Claw.close();
  
  turn_left(50, 30, true);
move_drive(550, 40, true);
//  move_lift(500, 80, true);

///////////////////////
move_drive(200, 20, true);
    turn_right(150, 20, true);
      move_mogo(515, 100, true);
  move_drive(225, 40, true);
      move_mogo(-515, 75, true);
      move_ring(1250, 100, true);





}







     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //             Driver Control             //
 //                                        //
//                                        //
///////////////////////////////////////////

void usercontrol(void) {


/////////////////////
//    Settings    //
///////////////////

bool mogo_up = false;
int mogo_lock = 0;
//mogo statements 

bool claw_up = true; 
int claw_lock = 0; 
  
bool rev_drive = false;
int rev_lock = 0;

double turnImportance = 0.5;


  while (1) {

set_coast();


    
    
    
    
/////////////////////////////////
//    Arcade Drive in Volts   //
///////////////////////////////

double turnPct = Controller1.Axis4.position();
double forwardPct = Controller1.Axis2.position();

double turnVolts = turnPct * 0.12; //sets the controller axis position from percent to voltage (voltage is out of 12)
double forwardVolts = forwardPct * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance); //multipled by turnImportance to have a less dramatic effect from turning while driving forward


  if (Controller1.ButtonB.pressing() && rev_lock==0){
    rev_drive = !rev_drive;
    rev_lock = 1;}

    else if (!Controller1.ButtonB.pressing()){
      rev_lock = 0;}

        if (rev_drive){
FR.spin(reverse, forwardVolts - turnVolts, voltageUnits::volt);
FL.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);
BR.spin(reverse, forwardVolts - turnVolts, voltageUnits::volt);
BL.spin(reverse, forwardVolts + turnVolts, voltageUnits::volt);
}

        else{
FR.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
FL.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
BR.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
BL.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
}

    


    
    
 /////////////////
//    Claw    //
///////////////

if (Controller1.ButtonR1.pressing() && claw_lock==0){
  claw_up = !claw_up; // flips the claw_up from true to false   or   false to true
  
  claw_lock = 1;} // claw_lock does not effect the motor
    //it is used to make sure that the above if statement runs just once 
    // (if you hold the button and dont have the lock, the if statement will run multiple times
        //causing claw up to continiously flip between true or false.

  else if (!Controller1.ButtonR1.pressing()){ //if R1 is not being pressed, set claw lock to 0 so the if statement can run again
    claw_lock = 0;}

    if(claw_up) //if claw_up is true, close the pneumatic piston on the claw
      Claw.close();

    else //if claw_up is false, open the pneumatic piston on the claw
      Claw.open();
    
    
    
    
    
    
////////////////////////
//    Ring Intake    //
//////////////////////

   
  if(Controller1.ButtonL1.pressing()){ //if L1 is pressing run the ring intake fwd
    ring();
}
 
    else if(Controller1.ButtonL2.pressing()){ //if L2 is pressing run the ring intake rev
      ringRev();
}

      else{ //if L2 && L1 not being pressed brake ring intake
        ringBrake(); 
}

    
    
    
    
    
///////////////////////////////
//    Mobile Goal Intake    //
/////////////////////////////

    // same logic as Claw
if (Controller1.ButtonR2.pressing() && mogo_lock==0){
  mogo_up = !mogo_up;
  mogo_lock = 1;}

  else if (!Controller1.ButtonR2.pressing()){
    mogo_lock = 0;}

      if (mogo_up)
        set_mogo_position(MOGO_OUT, 80);

      else
        set_mogo_position(MOGO_IN, 80);









    wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
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
