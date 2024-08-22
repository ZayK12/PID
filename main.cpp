#pragma region VEXcode Generated Robot Configuration
/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       27kazi-zayyaan                                            */
/*    Created:      8/22/2024, 8:33:28 AM                                     */
/*    Description:  V5 project                                                */
/*                                                                            */
/*----------------------------------------------------------------------------*/
#include "vex.h"

using namespace vex;

// A global instance of vex::brain used for printing to the V5 brain screen
vex::brain       Brain;

// START V5 MACROS
#define waitUntil(condition)                                                   \
  do {                                                                         \
    wait(5, msec);                                                             \
  } while (!(condition))

#define repeat(iterations)                                                     \
  for (int iterator = 0; iterator < iterations; iterator++)
// END V5 MACROS


// Robot configuration code.
controller Controller1 = controller(primary);
motor FrontLeft = motor(PORT20, ratio18_1, false);

motor BackLeft = motor(PORT10, ratio18_1, false);

motor FrontRight = motor(PORT11, ratio18_1, true);

motor BackRight = motor(PORT2, ratio18_1, true);

inertial Inertial4 = inertial(PORT4);


motor_group leftside = motor_group(FrontLeft, BackLeft);
motor_group rightside = motor_group(FrontRight, BackRight);

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
#pragma endregion VEXcode Generated Robot Configuration
competition Competition;
int Brain_precision = 0, Console_precision = 0;
int TV = 500;
/*
float MoveLateral(float x)
{
  float G = x/(3.14159 * 104.775) * 360;
  Brain.Screen.print(G);
  TV = G;
  return 0;
}
*/
int turninput = 0;

bool swtch = true; // switch to toggle while loop
bool swtch2 = true; // debug switch



#pragma region Tuning

//Lateral Tuning

double Kp = 0.275; // Constant for Proportion
double Ki = 0.0001; // Constant for Integral
double Kd = 0.0001;// Constant for Derivative

//Turn Tuning

double turnKp = 0.3; // Constant for Proportion
double turnKi = 0.000; // Constant for Integral
double turnKd = 0.000;// Constant for Derivative



#pragma endregion Tuning



#pragma region Declarables
float leftError = .1; //error is the distance left from the current position to the TV
float leftIntegral = 0;
float leftDerivative = 0;
float leftPrev_error = 0;
float rightError = .1;
float rightIntegral = 0;
float rightDerivative = 0;
float rightPrev_error = 0;
float prev_error = 0; // This is just the previous error before the next loop
float integral = 0; // 
float derivative = 0; // error - prev_error : 
float averagerot = 0; //
event getVar = event();
event reset = event();
float turnV = 0;
float turnError = .1; //error is the distance left from the current position to the TV
float turnIntegral = 0;
float turnDerivative = 0;
float turnPrev_error = 0;
float AVGRMP;
float AVGLMP;
bool AC = false;
#pragma endregion Declarables 


int ondriver_drivercontrol_0() 
{
  return 0;
}

void VEXcode_driver_task() {
  // Start the driver control tasks....
  vex::task drive0(ondriver_drivercontrol_0);
  while(Competition.isDriverControl() && Competition.isEnabled()) {this_thread::sleep_for(10);}
  drive0.stop();
  return;
}

void reset1()
{
  FrontRight.setPosition(0, degrees);
  BackRight.setPosition(0, degrees);
  FrontLeft.setPosition(0, degrees);
  BackLeft.setPosition(0, degrees);
  
}
#pragma region SIDPID

// Left calculation
void onevent_getVar_0()
{
  AVGLMP = FrontLeft.position(degrees) + BackLeft.position(degrees)/2;
  (TV != 0) ? leftError = TV - AVGLMP : leftError = 0;
  if (leftError == leftPrev_error)
  {
    leftIntegral = 0;
  }
  leftIntegral += leftError;
  leftPrev_error = leftError;
}

// Right Calculation

void onevent_getVar_1()
{
  AVGRMP = FrontRight.position(degrees) + BackRight.position(degrees)/2;
  (TV != 0) ? rightError = TV - AVGRMP : rightError = 0;
  if (rightError == rightPrev_error)
  {
    rightIntegral = 0;
    AC = true;
  }
  rightIntegral += rightError;
  rightPrev_error = rightError;
}
#pragma endregion SIDPID

//Turn Calculation

void onevent_getVar_2()
{
  averagerot = (AVGRMP + AVGLMP/2);
  (turninput != 0) ? turnError = (averagerot-turninput): turnError = 0;
  if (turnError == turnPrev_error)
  {
    turnIntegral = 0;
    //AC = true;
  } 
  turnIntegral += turnError;
  turnPrev_error = turnError;
  

}


void whenstarted1()
{ 
  while (swtch) 
  {
    turnV = turnKp*turnError + turnKi*turnIntegral + turnKd * (turnError - turnPrev_error);
    float rightlateralMotorPower = ((Kp*rightError + Ki*rightIntegral + Kd * (rightError - rightPrev_error))/12);
    float leftlateralMotorPower = ((Kp*leftError + Ki*leftIntegral + Kd * (leftError - leftPrev_error))/12);
    
    if (rightlateralMotorPower > float(10))
    
    {
      rightlateralMotorPower = 10;
      //return 0;
    }


    if (leftlateralMotorPower > float(10))
    {
      leftlateralMotorPower = 10;
      //return 0;
    }
    if (turnV > float(10))
    {
      turnV = 10;
    }
    leftside.spin(forward, leftlateralMotorPower + turnV, voltageUnits::volt);
    rightside.spin(forward, rightlateralMotorPower - turnV, voltageUnits::volt);
    //speed = Kp*error + Ki*integral + Kd * derivative;
    //return 0;
  } 

}

int whenStarted2()
{
  while (swtch2)
  {
    getVar.broadcast();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    Controller1.Screen.print(rightError);
    //printf(printToConsole_numberFormat(), static_cast<float>(turnError));
    //Controller1.Screen.print(" ");
    //Controller1.Screen.print(BackRight.position(degrees));
  }
  return 0;



}



int onauton_autonomous_0()
{
  wait(1, sec);
  AC = false;
  waitUntil(AC = true);
  reset1();
  TV = 300;
  return 0;
}



void VEXcode_auton_task() {
  // Start the auton control tasks....
  vex::task auto0(onauton_autonomous_0);
  while(Competition.isAutonomous() && Competition.isEnabled()) {this_thread::sleep_for(10);}
  auto0.stop();
  return;

}
int main() {
  getVar(onevent_getVar_0);
  getVar(onevent_getVar_1);
  getVar(onevent_getVar_2);
  reset(reset1);
  vex::competition::bStopTasksBetweenModes = false;
  Competition.drivercontrol(VEXcode_driver_task);
  Competition.autonomous(VEXcode_auton_task);
  swtch = true;
  swtch2 = true;
  vex::task ws1(whenStarted2);
  whenstarted1();

}