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
//change the number to alter the speed of the lift when going up (both numbers must be the same!!!)

void ringRev() {
  Ring.spin(directionType::rev, 11, voltageUnits::volt);}
//change the number to alter the speed of the lift when going down (both numbers must be the same!!!)

void ringBrake() {
  Ring.stop(brakeType::hold);}
//Lift brake types (hold,coast,brake)












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



//moves mogo (moves in degrees)
void move_mogo(int pos, int speed, bool stopping) {
  Mogo.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}



//rotates the ring intake
void move_ring(int pos, int speed, bool stopping) {
  Ring.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}






///////////////////////////////
//    PD Linear Movement    //
/////////////////////////////

void move_drive(double target, int speed){

set_position(0); // ^ zeros the Motors

double kp = 0; //speed control (error) 
double kd = 0; //makes sure that the speed is not too fast or too slow
// ^ constants


int error; //error (target - average position)
int prevError; //the error from the previous loop

int proportion; //error
int derivative; //error minus previous error (speed)  

int rawPow; //power calculated from summing the PID and their corresponding constants
int curDeg = (((FL.position(degrees) + FR.position(degrees))/2)); //average position between FL and FR
double curPos = curDeg * 0.047269;
int volts = (speed*.12); //converts the speed into voltage (0-12)

int sign; //sign (1 or -1) error/abs(error)

int DELAY_TIME = 10;
int velTimer;
int errorTimer;
int Lvel = FL.velocity(pct);
int Rvel = FR.velocity(pct);


while(1){

error = curPos - target;

//PD calculations using the average position of the motors
  proportion = error;
  derivative = (error - prevError);
    rawPow = proportion *kp + derivative *kd;



      sign = error / abs(int(error)); //calulates the sign of error (1 or -1)

if (abs(rawPow) >= abs(volts)){ // if the rawPower is greater than the desired speed, the rawPower equals the desired speed in volts times the sign of the error
  rawPow = volts*sign;
}


/*
if(abs(error) <= 4){
    errorTimer +=DELAY_TIME;
  if (errorTimer > 60){
    errorTimer = 0;
    break;
  }
}
  else{
    errorTimer = 0;
  }


if(Lvel == 0 && Rvel == 0){ //breaks the loop if the error is less than 4
    velTimer += DELAY_TIME;
  if (velTimer > 200) {
    velTimer = 0;
    break;
  }
}
else {
  velTimer = 0;
}

*/

//sets motors to move with the rawPower calculated from the PID controller
FL.spin(fwd, rawPow, vex::voltageUnits::volt);
FR.spin(fwd, rawPow, vex::voltageUnits::volt);
BL.spin(fwd, rawPow, vex::voltageUnits::volt);
BR.spin(fwd, rawPow, vex::voltageUnits::volt);

  prevError = error;
    wait(DELAY_TIME,msec); // waits 10 msec before repeating the while loop
} 


brake_drive();
  wait(20,msec);  //waits 20msec before going to the next line of code
}






/////////////////////////////////////
//    PID for Turning Movement    //
///////////////////////////////////

//p-loop for the inertial sensor
//sum two values
void turn_drive(int turnTarget, int speed, int turnType){

set_position(0); // ^ zeros the Motors and the Inertial Sensor

double kp = 0; //speed control (error) 
double ki = 0; //increase precision with error left over from kp 
double kd = 0; //makes sure that the speed is not too fast or too slow
// ^ constants (increase the kp until error is small, then increase kd until overshoot is minimal, increase ki until error is gone)

int error; //error (target - the actual values)
int prevError; //error from the previous loop

int proportion; //error
int integral = 0.0; //total error
int derivative; //error minus previous error (speed)

int curDeg = IMU.rotation(degrees);
int volts = (speed*.12); //converts the set speed above into an interger to volts (100 *.12 = 12)
  //voltageUnits::volts goes from 0-12 volts, 12 being the maximum voltage
int rawPow; //power calculated from summing the PID and their corresponding constants
int leftPower; //rawPow added with the difference between FL and FR
int rightPower; //rawPow subtracted with the difference between FL and FR
int sign; //interger that can only be 1 or -1 (used for the speed cap)

int DELAY_TIME = 10;
int velTimer;
int errorTimer;
int Lvel = FL.velocity(pct);
int Rvel = FR.velocity(pct);

while(1){ //while loop, this will continue to run until the specific parameters are met to break the loop



error = turnTarget - curDeg; //might have to be (actual - target) if it continues to turn after desired target

  proportion = error;
  integral += error;
  derivative = (error - prevError);
    rawPow = proportion *kp + integral *ki + derivative *kd;



  sign = error/abs(int(error)); //calulates the sign of error (1 or -1)

if(abs(rawPow)>=abs(volts)){ // if the left power is greater than the desired speed, the left power equals the desired speed in volts times the sign of the error
  rawPow = volts*sign; // power left side = speed times sign of the error (direction)
}



if (turnType == 0){ //turn
    leftPower = -rawPow;
    rightPower = rawPow;
}

if (turnType == 1){ //left swing
    leftPower = 0;
    rightPower = rawPow;
}

if (turnType == 2){ //right swing
    leftPower = rawPow;
    rightPower = 0;
}


/*
if(abs(error) <= 4){ //breaks the loop if the error is less than 4 for more than 60 msec
    errorTimer +=DELAY_TIME;
  if (errorTimer > 60){
    errorTimer = 0;
    break;
  }
}
  else{
    errorTimer = 0;
  }


if(Lvel == 0 && Rvel == 0){ //breaks the loop if the velocity is 0 for more than 200 msec
    velTimer += DELAY_TIME;
  if (velTimer > 200) {
    velTimer = 0;
    break;
  }
}
else {
  velTimer = 0;
}
*/


//sets motors to move with their corresponding powers
FL.spin(fwd, leftPower, vex::voltageUnits::volt);
FR.spin(fwd, rightPower, vex::voltageUnits::volt);
BL.spin(fwd, leftPower, vex::voltageUnits::volt);
BR.spin(fwd, rightPower, vex::voltageUnits::volt);

  prevError = error; 
    wait(DELAY_TIME,msec); // waits 10msec before repeating the while loop
} 



//coasts the motors when while loop broken
brake_drive();
  wait(20,msec); //waits 20msec before going to the next line of code
}












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

    IMU.calibrate();
    reset_rotation();
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
    reset_rotation();


















/*
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
*/




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

double turnVolts = turnPct * 0.12;
double forwardVolts = forwardPct * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);


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



////////////////////////
//    Ring Intake    //
//////////////////////

  if(Controller1.ButtonL1.pressing()){
    ring();
}

    else if(Controller1.ButtonL2.pressing()){
      ringRev();
}

      else{
        ringBrake();
}






///////////////////////////////
//    Mobile Goal Intake    //
/////////////////////////////

if (Controller1.ButtonR2.pressing() && mogo_lock==0){
  mogo_up = !mogo_up;
  mogo_lock = 1;}

  else if (!Controller1.ButtonR2.pressing()){
    mogo_lock = 0;}

      if (mogo_up)
        set_mogo_position(MOGO_OUT, 80);

      else
        set_mogo_position(MOGO_IN, 80);






/////////////////
//    Claw    //
///////////////

if (Controller1.ButtonR1.pressing() && claw_lock==0){
  claw_up = !claw_up;
  claw_lock = 1;}

  else if (!Controller1.ButtonR1.pressing()){
    claw_lock = 0;}

    if(claw_up)
      Claw.close();

    else
      Claw.open();












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
