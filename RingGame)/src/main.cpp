/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       William Wang                                              */
/*    Created:      Fri May 28 2021                                           */
/*    Description:  2021 - 2022 Vex game program                              */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

controller Controller1;

motor chain (PORT4, ratio18_1);
motor deflector (PORT5, ratio18_1);

motor lBack (PORT1, ratio18_1);
motor lFront (PORT2, ratio18_1);

motor rBack (PORT11, ratio18_1, true);
motor rFront (PORT12, ratio18_1, true);

motor lArm (PORT3, ratio18_1);
motor rArm (PORT13, ratio18_1, true);


motor_group lTrain (lBack, lFront);
motor_group rTrain (rBack, rFront);
motor_group arm (lArm, rArm);

drivetrain Train (lTrain, rTrain);

float frontSpeed = 1  ;
float backSpeed = 1;
float driveSpeed = 1;
float armSpeed = 70;
float chainSpeed = 80;
float deflectorSpeed = 50;

//settings
double kP = 0.25;
double kI = 0.0;
double kD = 0.1;

double turnkP = 0.2; 
double turnkI = 0.0;
double turnkD = 0.1;

//autonomous settings
int desiredValue = 20;
int desiredTurnValue = 0;

int error;//sensorvalue - desiredvalue : position
int prevError = 0; //postion 20 msec ago
int derivative;//error - preverror : speed
int totalError;

int turnError;//sensorvalue - desiredvalue : position
int turnPrevError = 0; //postion 20 msec ago
int turnDerivative;//error - preverror : speed
int turnTotalError;

const float WHEEL_CIRCUMFERENCE = 31.9185812596;
const float MOTOR_ACCEL_LIMIT = 8;

int s_lastL = 0;
int s_lastR = 0; 

// define your global instances of motors and other devices here

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
void DriveDistance(int dist, float maxTime)
{
  lBack.resetPosition();
  rBack.resetPosition();
  lFront.resetPosition();
  rFront.resetPosition();

  //Constant Tuning Values
  const float Kp = 1;
  const float Kd = 0;
  const float Ki = 0;

  float rotationGoal = (dist / WHEEL_CIRCUMFERENCE) * 360;




  float distError = 0;
  float integral = 0;
  float derivative = 0;
  float lastError = 0;

  float motorSpeed = 0;
  
  float doneTime = 0;
  while(maxTime > doneTime / 1000)
  {
    distError = rotationGoal - lFront.rotation(deg);

    integral += distError;

    if(distError > 200 || distError < -200)
    {
      integral  = 0;
    }

    derivative = distError - lastError;

    lastError = distError;

    motorSpeed = Kp * distError + Ki * integral + Kd * derivative;
    lTrain.spin(forward, motorSpeed, pct);
    rTrain.spin(forward, motorSpeed, pct);
  }
}


void UpdateScreen() {
  Controller1.Screen.clearScreen();

  if (Competition.isDriverControl()) 
  {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("User Control Active");
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Battery: ");
    Controller1.Screen.print(Brain.Battery.capacity());
    Controller1.Screen.print("%%");
  }
  else if (Competition.isAutonomous()) {
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1, 1);
    Controller1.Screen.print("Autonomous");
    Controller1.Screen.setCursor(3, 1);
    Controller1.Screen.print("Battery: ");
    Controller1.Screen.print(Brain.Battery.capacity());
    Controller1.Screen.print("%%");
  }
}

void reverseDrive() {



    lTrain.spin(vex::directionType::fwd, Controller1.Axis3.position(vex::percentUnits::pct) * driveSpeed, vex::velocityUnits::pct);
    rTrain.spin(vex::directionType::fwd, Controller1.Axis2.position(vex::percentUnits::pct) * driveSpeed, vex::velocityUnits::pct);

    //Stops the motor if the controller joystick's position is equal to 0
    if(Controller1.Axis2.position() == 0){
      rTrain.stop(vex::brake);
    }
    else if(Controller1.Axis3.position() == 0){
      lTrain.stop(vex::brake);
    }
    else if(Controller1.Axis2.position() == 0 && Controller1.Axis3.position() == 0){
      rTrain.stop(vex::brake);
      lTrain.stop(vex::brake);
    }
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
    //DriveDistance(10, 4);
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
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.
    
    //arm commands
    if(Controller1.ButtonL1.pressing()){
      lArm.spin(reverse, armSpeed, pct);
      rArm.spin(reverse, armSpeed, pct);
    }  
    else if(Controller1.ButtonL2.pressing()){
      lArm.spin(fwd, armSpeed, pct);
      rArm.spin(fwd, armSpeed, pct);
    }
    else{
      lArm.spin(fwd, 0.3, pct);
      rArm.spin(fwd, 0.3, pct);
    }
     
    //chain spin
    if(Controller1.ButtonR1.pressing()){
      chain.spin(fwd, chainSpeed, pct);
    }
    else if (Controller1.ButtonR2.pressing()){
      chain.spin(reverse, chainSpeed,pct);
    }
    else {
      chain.stop(vex::brake);
    }

    //defelctor spin
    if(Controller1.ButtonUp.pressing()){
      deflector.spin(fwd, deflectorSpeed, pct);
    }
    else if(Controller1.ButtonDown.pressing()){
      deflector.spin(reverse, deflectorSpeed, pct);
    }
    else{
      deflector.spin(fwd, 0.5, pct);
    }

    //limit the turning speed of the motors
   //if(Controller1.Axis2.position()-Controller1.Axis3.position()>50){
     // driveSpeed = 0.70;
    //}
    //else if (Controller1.Axis3.position()-Controller1.Axis2.position()<=50){
      //driveSpeed = 0.70;
    //}
    //else{
      //driveSpeed = 1;
    //}

    //drive commands
    lTrain.spin(vex::directionType::fwd, Controller1.Axis3.position(vex::percentUnits::pct) * driveSpeed, vex::velocityUnits::pct);
    rTrain.spin(vex::directionType::fwd, Controller1.Axis2.position(vex::percentUnits::pct) * driveSpeed, vex::velocityUnits::pct);

    //Stops the motor if the controller joystick's position is equal to 0
    if(Controller1.Axis2.position() == 0){
      rTrain.stop(vex::brake);
    }
    else if(Controller1.Axis3.position() == 0){
      lTrain.stop(vex::brake);
    }
    else if(Controller1.Axis2.position() == 0 && Controller1.Axis3.position() == 0){
      rTrain.stop(vex::brake);
      lTrain.stop(vex::brake);
    }

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
  UpdateScreen();

  Controller1.ButtonA.pressed(reverseDrive);
  Controller1.ButtonB.pressed(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
