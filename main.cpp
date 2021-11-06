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
// Lift2                motor         12              
// Tilter               motor         18              
// IS                   inertial      1               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include <cmath>

using namespace vex;

// A global instance of competition
competition Competition;



// define your global instances of motors and other devices here


//Autonomous Code Global Statements

//reset sensors so they are more accurate
void reset_rotation() {
  FR.resetRotation(); BR.resetRotation(); FL.resetRotation(); BL.resetRotation(); Claw.close(); LeftLift.resetRotation(); RightLift.resetRotation(); }

void set_position(int pos){
  FL.setPosition(pos, degrees);
  FR.setPosition(pos, degrees);
  BL.setPosition(pos, degrees);
  BR.setPosition(pos, degrees);
  IMU.setHeading(pos, degrees);
}


//brakes drive in either hold or coast
void set_hold() {
  FR.setStopping(hold); BR.setStopping(hold); FL.setStopping(hold); BL.setStopping(hold); }

void set_coast() {
  FR.setStopping(coast); BR.setStopping(coast); FL.setStopping(coast); BL.setStopping(coast); }

void coast_drive(){
FL.stop(coast); FR.stop(coast); BL.stop(coast); BR.stop(coast); }

void brake_drive(){
FL.stop(hold); FR.stop(hold); BL.stop(hold); BR.stop(hold); }

//These Autonomous Global Statements use the internal motor's encoders to run
//Uses the rotateFor command, which makes the robot move for a specific amount of deg or rev
//Uses boolean statement (true or false) to decide whether the code will run the next line immediately or wait for the line to finish





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
/*
if (IMU.heading(degrees)>=180){
  curDeg = IMU.heading(degrees) - 360;
}
*/
//PID Calculations using the inertial sensor
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






///////////////////////////////
//    PD Linear Movement    //
/////////////////////////////

void move_drive(int target, int speed){

  FL.setPosition(0, degrees);
  FR.setPosition(0, degrees);
  BL.setPosition(0, degrees);
  BR.setPosition(0, degrees);// ^ zeros the Motors

double kp = 0.07; //speed control (error) 0.185
double kd = 0.0585395; //makes sure that the speed is not too fast or too slow 0.17
double turnKp = 0.025;
double turnKd = 0.011875;
// ^ constants


int error; //error (target - average position)
int prevError; //the error from the previous loop
int turnError;
int turnPrevError;

int proportion; //error
int derivative; //error minus previous error (speed)
int turnProportion;
int turnDerivative;

int rawPow; //power calculated from summing the PID and their corresponding constants
int curPos;
int turnRawPow;
int turnDiff;
int leftPow;
int rightPow;

//double curPos = curDeg * 0.047269;
int volts = (speed*.12); //converts the speed into voltage (0-12)

int sign; //sign (1 or -1) error/abs(error)

int DELAY_TIME = 10;
int velTimer;
int errorTimer;
int Lvel = FL.velocity(pct);
int Rvel = FR.velocity(pct);


while(1){

curPos = ((FL.position(degrees) + FR.position(degrees))/2); //average position between FL and FR
turnDiff = (FL.position(degrees) - FR.position(degrees));

error = target - curPos;

//PD calculations using the average position of the motors
  proportion = error;
  derivative = (error - prevError);
    rawPow = proportion *kp + derivative *kd;
printf("%i", rawPow);


turnError = 0 - turnDiff;

  turnProportion = turnError;
  turnDerivative = (turnError - turnPrevError);
    turnRawPow = turnProportion *turnKp + turnDerivative *turnKd;

leftPow = rawPow + turnRawPow;
rightPow = rawPow - turnRawPow;


/*
      sign = error / abs(int(error)); //calulates the sign of error (1 or -1)

if (abs(rawPow) >= abs(volts)){ // if the rawPower is greater than the desired speed, the rawPower equals the desired speed in volts times the sign of the error
  rawPow = volts*sign;
}
*/

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
FL.spin(fwd, leftPow, vex::voltageUnits::volt);
FR.spin(fwd, rightPow, vex::voltageUnits::volt);
BL.spin(fwd, leftPow, vex::voltageUnits::volt);
BR.spin(fwd, rightPow, vex::voltageUnits::volt);


  prevError = error;
  turnPrevError = turnError;
    wait(DELAY_TIME,msec); // waits 10 msec before repeating the while loop
} 


brake_drive();
  wait(20,msec);  //waits 20msec before going to the next line of code
}














void move_drive_test(int pos, int speed){
  FR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  FL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BR.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, false);
  BL.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, true);

}



/*    constant tuning tips from Jess
set all constant values to zero. Increase the Kp value until there is steady oscillation
then increase the Kd value until the oscillation is gone
repeat until increasing the Kd value does stop the oscillation. Then go back to the last Kd and Kp value with no oscillation
then (for turns) increase the Ki so the final error is gone. 
Increase your gains and set turnKi and turnKd to 0 –– just a heads up.    */






///////////////////////////////////////////
//    Proportion - Loop for the Lift    //
/////////////////////////////////////////

void move_lift(int target, int speed){

double kp = 0.0;

int error;
int proportion;
int rawPow;
int volts = (speed * 0.12);
int DELAY_TIME = 10;
int errorTimer;
int sign;

while(1){
  error = target - Pot.value(degrees);

  proportion = error;

  rawPow = proportion * kp;


  sign = error / abs(int(error));

  if (abs(rawPow) >= abs(volts)){
    rawPow = volts*sign;
  }

  if(abs(error) < 4){
    errorTimer += DELAY_TIME;

    if (errorTimer > 60){
      errorTimer = 0;
      break;
    } 
  }

LeftLift.spin(fwd, rawPow, vex::voltageUnits::volt);
RightLift.spin(fwd, rawPow, vex::voltageUnits::volt);
}

wait(DELAY_TIME,msec);
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
reset_rotation();

move_drive(508, 100);

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
  while (1) {
    
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
