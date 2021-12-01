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
  FR.setStopping(hold); MR.setStopping(hold); BR.setStopping(hold); FL.setStopping(hold); ML.setStopping(hold); BL.setStopping(hold); }

void set_coast() {
  FR.setStopping(coast); MR.setStopping(coast); BR.setStopping(coast); FL.setStopping(coast); ML.setStopping(coast); BL.setStopping(coast); }

void coast_drive(){
FR.stop(coast); MR.stop(coast); BR.stop(coast); FL.stop(coast); ML.stop(coast); BL.stop(coast); }

void brake_drive(){
FR.stop(hold); MR.stop(hold); BR.stop(hold); FL.stop(hold); ML.stop(hold); BL.stop(hold); }

void reset_rotation() {
  FR.resetRotation(); MR.resetRotation(); BR.resetRotation(); FL.resetRotation(); ML.resetRotation(); BL.resetRotation(); Lift.resetRotation(); Ring.resetRotation(); 
  LiftClaw.open(); Dock.open(); Slift.close(); Slift.close(); }

void set_position(int pos){
  FR.setPosition(pos, deg); MR.setPosition(pos, deg); BR.setPosition(pos, deg); FL.setPosition(pos, deg); MR.setPosition(pos, deg); BL.setPosition(pos, deg); }












     ////////////////////////////////////////////
    //                                        //
   //                                        //
  //    Driver Control Global Statements    //
 //                                        //
//                                        //
///////////////////////////////////////////





////////////////////////////
//    Ring Statements    //
//////////////////////////

void ring() { //85.416667  .91666667
  Ring.spin(directionType::fwd, 12, voltageUnits::volt);}
//change the number to alter the speed of the lift when going up (both numbers must be the same!!!)

void ringRev() {
  Ring.spin(directionType::rev, 12, voltageUnits::volt);}
//change the number to alter the speed of the lift when going down (both numbers must be the same!!!)

void ringBrake() {
  Ring.stop(brakeType::hold);}
//Lift brake types (hold,coast,brake)






/////////////////////////////
//    4-Bar Statements    //
///////////////////////////

void Small_Lift_Open() {
  Slift.open();
}

void Small_Lift_Close() {
  Slift.close();
}




////////////////////////////
//    Lift Statements    //
//////////////////////////

void lift() { 
  Lift.spin(directionType::fwd, 11, voltageUnits::volt);}
//change the number to alter the speed of the lift when going up (both numbers must be the same!!!)

void liftRev() {
  Lift.spin(directionType::rev, 11, voltageUnits::volt);}
//change the number to alter the speed of the lift when going down (both numbers must be the same!!!)

void liftBrake() {
  Lift.stop(brakeType::hold);}
//Lift brake types (hold,coast,brake)









     ////////////////////////////////////////
    //                                    //
   //                                    //
  //    Autonomous Global Statements    //
 //                                    //
//                                    //
///////////////////////////////////////

const int HALF_SPEED = 50;
const int FULL_SPEED = 100;
const int QUARTER_SPEED = 25;
const int NO_MIN = 0;



/////////////////////////////////////
//    PID for Turning Movement    //
///////////////////////////////////

void turn_drive(int turnTarget, int speed){

set_position(0); // ^ zeros the Motors and the Inertial Sensor

double kp = 1.0; //speed control (error) 
double ki = 0.0; //increase precision with error left over from kp 
double kd = 8.3; //makes sure that the speed is not too fast or too slow
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

int DELAY_TIME = 10;
int velTimer;
int errorTimer;
int Lvel = FL.velocity(pct);
int Rvel = FR.velocity(pct);

while(1){ //while loop, this will continue to run until the specific parameters are met to break the loop

//PID Calculations using the inertial sensor
error = turnTarget - IMU.rotation(degrees); //might have to be (actual - target) if it continues to turn after desired target


  proportion = error;
  integral += error;
  derivative = (error - prevError);
    rawPow = proportion *kp + integral *ki + derivative *kd;




  sign = error/abs(int(error)); //calulates the sign of error (1 or -1)

if(abs(rawPow)>=abs(volts)){ // if the left power is greater than the desired speed, the left power equals the desired speed in volts times the sign of the error
  rawPow = volts*sign; // power left side = speed times sign of the error (direction)
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






//sets motors to move with their corresponding powers
FL.spin(forward, rawPow, vex::voltageUnits::volt);
FR.spin(reverse, rawPow, vex::voltageUnits::volt);
ML.spin(forward, rawPow, vex::voltageUnits::volt);
MR.spin(reverse, rawPow, vex::voltageUnits::volt);
BL.spin(forward, rawPow, vex::voltageUnits::volt);
BR.spin(reverse, rawPow, vex::voltageUnits::volt);

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

void move_drive(int target, int max_speed, int min_speed){

set_position(0);
  // ^ zeros the Motors

double kp = 0.15; //speed control (error) 0.185
double kd = 1.51; //makes sure that the speed is not too fast or too slow 0.17
//double turnKp = 0.025;
//double turnKd = 0.011875;
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
int max_volts = (max_speed * .12); //converts the speed into voltage (0-12)
int min_volts = (min_speed * .12);

int sign; //sign (1 or -1) error/abs(error)

int DELAY_TIME = 10;
int velTimer;
int errorTimer;
int Lvel = FL.velocity(pct);
int Rvel = FR.velocity(pct);


while(1){

curPos = ((FL.position(degrees) + FR.position(degrees))/2); //average position between FL and FR
//turnDiff = (FL.position(degrees) - FR.position(degrees));

error = target - curPos;

//PD calculations using the average position of the motors
  proportion = error;
  derivative = (error - prevError);
    rawPow = proportion *kp + derivative *kd;
printf("%i", rawPow);


/*
turnError = 0 - turnDiff;

  turnProportion = turnError;
  turnDerivative = (turnError - turnPrevError);
    turnRawPow = turnProportion *turnKp + turnDerivative *turnKd;

leftPow = rawPow + turnRawPow;
rightPow = rawPow - turnRawPow;
*/


      sign = error / abs(int(error)); //calulates the sign of error (1 or -1)

if (abs(rawPow) <= abs(min_volts)){
  rawPow = min_volts*sign;
}

if (abs(rawPow) >= abs(max_volts)){ // if the rawPower is greater than the desired speed, the rawPower equals the desired speed in volts times the sign of the error
  rawPow = max_volts*sign;
}

/*
if (abs(leftPow) <= abs(min_volts)){
  leftPow = min_volts*sign;
}

if (abs(leftPow) >= abs(max_volts)){
  leftPow = min_volts*sign;
}


if (abs(rightPow) <= abs(min_volts)){
  leftPow = min_volts*sign;
}

if (abs(rightPow) >= abs(max_volts)){
  leftPow = min_volts*sign;
}

*/

if(abs(error) <= 5 ){   // && abs(turnError) <= 4
    errorTimer +=DELAY_TIME;
  if (errorTimer > 60){
    errorTimer = 0;
      break;
  }
}
  else{
    errorTimer = 0;
  }
/*

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
ML.spin(fwd, rawPow, vex::voltageUnits::volt);
MR.spin(fwd, rawPow, vex::voltageUnits::volt);
BL.spin(fwd, rawPow, vex::voltageUnits::volt);
BR.spin(fwd, rawPow, vex::voltageUnits::volt);


  prevError = error;
//  turnPrevError = turnError;
    wait(DELAY_TIME,msec); // waits 10 msec before repeating the while loop
} 


brake_drive();
  wait(20,msec);  //waits 20msec before going to the next line of code
}








//moves lift (moves in degrees)
void move_lift(int pos, int speed, bool stopping) {
  Lift.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
}



//rotates the ring intake
void move_ring(int pos, int speed, bool stopping) {
  Ring.rotateFor(pos, rotationUnits::deg, speed, velocityUnits::pct, stopping);
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

  bool mogo_up = true;
  int mogo_lock = 0;

  bool claw_up = false;
  int claw_lock = 0;

  bool dock_up = false;
  int dock_lock = 0;

  double turnImportance = 0.125;


while (1) {

  set_coast();

    
/////////////////////////////////
//    Arcade Drive in Volts   //
///////////////////////////////

double turnPct = Controller1.Axis4.position();
double forwardPct = Controller1.Axis2.position();

double turnVolts = turnPct * 0.12;
double forwardVolts = forwardPct * 0.12 * (1 - (std::abs(turnVolts)/12.0) * turnImportance);


FR.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
MR.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);
BR.spin(forward, forwardVolts - turnVolts, voltageUnits::volt);


FL.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
ML.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);
BL.spin(forward, forwardVolts + turnVolts, voltageUnits::volt);




////////////////////////
//    Ring Intake    //
//////////////////////

  if(Controller1.ButtonR1.pressing()){
    ring();
}

    else if(Controller1.ButtonR2.pressing()){
      ringRev();
}

      else{
        ringBrake();
}




////////////////
//    Lift   //
//////////////

  if(Controller1.ButtonL1.pressing()){
    lift();
}

    else if(Controller1.ButtonL2.pressing()){
      liftRev();
}

      else{
        liftBrake();
}





///////////////////////////////////
//    Mobile Goal Mini 4-Bar    //
/////////////////////////////////

if (Controller1.ButtonDown.pressing() && mogo_lock==0){
  mogo_up = !mogo_up;
  mogo_lock = 1;}

  else if (!Controller1.ButtonDown.pressing()){
    mogo_lock = 0;}

      if (mogo_up)
        Small_Lift_Open();
      
      else
        Small_Lift_Close();




      




/////////////////
//    Claw    //
///////////////

if (Controller1.ButtonRight.pressing() && claw_lock==0){
  claw_up = !claw_up;
  claw_lock = 1;}

  else if (!Controller1.ButtonRight.pressing()){
    claw_lock = 0;}

    if(claw_up)
      LiftClaw.close();

    else
      LiftClaw.open();






/////////////////
//    Dock    //
///////////////

if (Controller1.ButtonY.pressing() && dock_lock==0){
  dock_up = !dock_up;
  dock_lock = 1;}

  else if (!Controller1.ButtonY.pressing()){
    dock_lock = 0;}

    if(dock_up)
      Dock.close();

    else
      Dock.open();












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
