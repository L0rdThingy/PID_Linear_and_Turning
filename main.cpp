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

using namespace vex;

// A global instance of competition
competition Competition;



// define your global instances of motors and other devices here


//Autonomous Code Global Statements

//reset sensors so they are more accurate
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

void set_position(int pos){
  FL.setPosition(pos, degrees);
  FR.setPosition(pos, degrees);
  BL.setPosition(pos, degrees);
  BR.setPosition(pos, degrees);
  IS.setHeading(pos, degrees);
}


//brakes drive in either hold or coast
void brake_drive() {
  FR.setStopping(hold);
  BR.setStopping(hold);
  FL.setStopping(hold);
  BL.setStopping(hold);
}

void coast_drive() {
  FR.setStopping(coast);
  BR.setStopping(coast);
  FL.setStopping(coast);
  BL.setStopping(coast);
}


//These Autonomous Global Statements use the internal motor's encoders to run
//Uses the rotateFor command, which makes the robot move for a specific amount of deg or rev
//Uses boolean statement (true or false) to decide whether the code will run the next line immediately or wait for the line to finish





///////////////////////////////////////////////////////////////////////////////////////////////////////
//PID for Turning Movement
///////////////////////////////////////////////////

void turn_drive(int turnTarget, int speed){
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

int volts = (speed*.12); //converts the set speed above into an interger to volts (100 *.12 = 12)
  //voltageUnits::volts goes from 0-12 volts, 12 being the maximum voltage
int rawPow; //power calculated from summing the PID and their corresponding constants
int leftPower; //rawPow added with the difference between FL and FR
int rightPower; //rawPow subtracted with the difference between FL and FR
int sign; //interger that can only be 1 or -1 (used for the speed cap)



while(1){ //while loop, this will continue to run until the specific parameters are met to break the loop

//PID Calculations using the inertial sensor
error = turnTarget - IS.heading(degrees); //might have to be (actual - target) if it continues to turn after desired target
prevError = error;



  proportion = error;
  integral += error;
  derivative = (error - prevError);
    rawPow = proportion *kp + integral *ki + derivative *kd;


    leftPower = rawPow;
    rightPower = rawPow;


  sign = error/abs(int(error)); //calulates the sign of error (1 or -1)

if(leftPower>=volts){ // if the left power is greater than the desired speed, the left power equals the desired speed in volts times the sign of the error
  leftPower = volts*sign;}

if(rightPower>=volts){ // if the right power is greater than the desired speed, the right power equals the desired speed in volts times the sign of the error
  rightPower = volts*sign;}



if (error<4){ // if the error is less than 4 degrees then break out of the while loop
  break;}



//sets motors to move with their corresponding powers
FL.spin(reverse, leftPower, vex::voltageUnits::volt);
FR.spin(fwd, rightPower, vex::voltageUnits::volt);
BL.spin(reverse, leftPower, vex::voltageUnits::volt);
BR.spin(fwd, rightPower, vex::voltageUnits::volt);
  wait(20,msec);} // waits 20msecs before repeating the while loop



//coasts the motors when while loop broken
FL.stop(coast); FR.stop(coast); BL.stop(coast); BR.stop(coast);
  wait(20,msec);} //waits 20msec before going to the next line of code









//////////////////////
//PID Linear Movement
/////////////////////////////////////////


void move_drive(int target, int speed, int timeout){
Brain.Timer.reset();
set_position(0);// ^ zeros the Motors

double kp = 0; //speed control (error) 
double ki = 0; //increase precision with error left over from kp 
double kd = 0; //makes sure that the speed is not too fast or too slow
// ^ constants (increase the kp until error is small, then increase kd until overshoot is minimal, increase ki until error is gone)

int error; //error (target - average position)
int prevError; //the error from the previous loop

int proportion; //error
int integral = 0.0; //total error
int derivative; //error minus previous error (speed)

int rawPow; //power calculated from summing the PID and their corresponding constants
int avgPos = ((FL.position(degrees) + FR.position(degrees))/2); //average position between FL and FR
int volts = (speed*.12); //converts the speed into voltage (0-12)

int sign; //sign (1 or -1) error/abs(error)

int currentTime; // current time
int prevTime; // time from the previous loop
int deltaTime; // current time minus the previous time
// ^ uses time to make sure that the PID controller isnt running for longer than it should



while(1){
currentTime = Brain.Timer.value();
prevTime = currentTime;
deltaTime = (currentTime - prevTime);



error = target - avgPos;
prevError = error;



//PID calculations using the average position of the motors
  proportion = error;
  integral += error*(deltaTime);
  derivative = (error - prevError)/(deltaTime);
    rawPow = proportion *kp + integral *ki + derivative *kd;



      sign = error / abs(int(error)); //calulates the sign of error (1 or -1)

if (rawPow>=volts){ // if the rawPower is greater than the desired speed, the rawPower equals the desired speed in volts times the sign of the error
  rawPow = volts*sign;
}



if(Brain.Timer.value()>=timeout){ //breaks the loop if the timer value is greater or equal to the time set above
  break;}

if(error<4){ //breaks the loop if the error is less than 4
  break;}



//sets motors to move with the rawPower calculated from the PID controller
FL.spin(fwd, rawPow, vex::voltageUnits::volt);
FR.spin(fwd, rawPow, vex::voltageUnits::volt);
BL.spin(fwd, rawPow, vex::voltageUnits::volt);
BR.spin(fwd, rawPow, vex::voltageUnits::volt);
    wait(20,msec);} // waits 20msecs before repeating the while loop



Brain.Screen.print(currentTime); //used to figure out what to set the timeout value
FL.stop(coast); FR.stop(coast); BL.stop(coast); BR.stop(coast);
    wait(20,msec);} //waits 20msec before going to the next line of code

/*

    //constant tuning tips from vexforum
Increase your gains and set turnKi and turnKd to 0 –– just a heads up.
I probably should’ve mentioned you should always start with a high-value forkp and work down from there.
Make sure your motors are receiving power to and the function is actually executing.
Another tuning tip, keep lowering kp until the robot barely reaches the target.

*/




























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
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
reset_rotation();



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
