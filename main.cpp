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
#include <math.h>
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
motor FrontLeft = motor(PORT19, ratio18_1, false);

motor BackLeft = motor(PORT18, ratio18_1, true);

motor FrontRight = motor(PORT11, ratio18_1, false);

motor BackRight = motor(PORT12, ratio18_1, true);

inertial Inertial4 = inertial(PORT4);

//Sides for tank drive, corners on X-Drive
motor_group leftside = motor_group(FrontLeft, BackLeft);
motor_group rightside = motor_group(FrontRight, BackRight);

// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
#pragma endregion VEXcode Generated Robot Configuration
competition Competition;
int Brain_precision = 0, Console_precision = 0;

double TV = -500.0/(3.14159 * 104.775) * 360; // Targeted Value
// The calculation is used to convert mm to degrees.

double turninput = 0.0;
// Tank Drive turning X-Drive Side to side movement.

//X-Drive Turning, may work with mechan wheels (Untested)
double actualturning = 0.0;

//swtch toggles the PID calculations and spinning the motors
bool swtch = false; // switch to toggle while loop
//swtch2 toggles updating all PID variables
bool swtch2 = true; // debug switch



#pragma region Tuning

//Lateral Tuning

double Kp = 0.375; // Constant for Proportion
double Ki = 0.0001; // Constant for Integral
double Kd = 0.0001;// Constant for Derivative

//Turn Tuning

double turnKp = 0.3; // Constant for Proportion
double turnKi = 0.000; // Constant for Integral
double turnKd = 0.000;// Constant for Derivative

//Turning Tuning
double actualturningKp = 0.0;
double actualturningKi = 0.0;
double actualturningKd = 0.0;


#pragma endregion Tuning



#pragma region Declarables
double leftError = .1; //error is the distance left from the current position to the TV
double leftIntegral = 0.0;
double leftDerivative = 0.0;
double leftPrev_error = 0.0;

double rightError = .1;
double rightIntegral = 0.0;
double rightDerivative = 0.0;
double rightPrev_error = 0.0;

double prev_error = 0.0; // This is just the previous error before the next loop
double integral = 0.0; // 
double derivative = 0.0; // error - prev_error : 
double averagerot = 0.0; //

//getVar event is used to multithread the code
event getVar = event();
//reset event is ued to reset all PID calculations.
event reset12 = event();

//PID variables attached to the turninput variable
double turnV = 0.0;
double turnError = .1; //error is the distance left from the current position to the TV
double turnIntegral = 0.0;
double turnDerivative = 0.0;
double turnPrev_error = 0.0;

//Average Right Motor Position
double AVGRMP;

//Average Left Motor Position
double AVGLMP;

//Unused all clear boolean for resettting
bool AC = false;
//Variables to track what movement method is in use
bool longLat = false;
bool Turn = false;

//actual turning is for x-drive turning as turn causes side to side movement
double actualturningV = 0.0; //Voltage to give motors
double actualturningIntegral = 0.0;
double actualturningDerivative = 0.0;
double actualturningPrev_error = 0.0;
double actualturningError = 0.0;
double turningrot = 0.0;


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

double moveLong(double input)
{
  double TV = input/(3.14159 * 104.775) * 360; // Targeted Value
  longLat = true;
  return TV;
}
double turn(double input)
{
  turninput = input;
  Turn = true;
  return 0;
}
#pragma region SIDPID


// Left calculation
void onevent_getVar_0()
{
  AVGLMP = FrontLeft.position(degrees) + BackLeft.position(degrees)/2;
  (TV != 0.0) ? leftError = TV - AVGLMP : leftError = 0.0;
  if (leftError == leftPrev_error)
  {
    leftIntegral = 0.0;
  }
  leftIntegral += leftError;
  leftPrev_error = leftError;
}

// Right Calculation

void onevent_getVar_1()
{
  AVGRMP = FrontRight.position(degrees) + BackRight.position(degrees)/2;
  (TV != 0.0) ? rightError = TV - AVGRMP : rightError = 0.0;
  if (rightError == rightPrev_error)
  {
    rightIntegral = 0.0;
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
  (turninput != 0.0) ? turnError = (averagerot-turninput): turnError = 0.0;
  if (turnError == turnPrev_error)
  {
    turnIntegral = 0.0;
    //AC = true;
  } 
  turnIntegral += turnError;
  turnPrev_error = turnError;
  
  
}

//Calculation used for X-Drive turning.
void onevent_getVar_3()
{
  turningrot = fabs(FrontRight.position(degrees)) +fabs(BackRight.position(degrees));
  (actualturning != 0.0) ? actualturning = (averagerot-actualturning): actualturningError = 0.0;
  if (actualturningError == actualturningPrev_error)
  {
    actualturningIntegral = 0.0;
  }
  actualturningIntegral += actualturningError;
  actualturningPrev_error = actualturningError;

}

//Final Calculation 
void onevent_getVar_4()
{
  while (swtch) 
  {
    turnV = turnKp*turnError + turnKi*turnIntegral + turnKd * (turnError - turnPrev_error);
    double rightlateralMotorPower = ((Kp*rightError + Ki*rightIntegral + Kd * (rightError - rightPrev_error))/12);
    double leftlateralMotorPower = ((Kp*leftError + Ki*leftIntegral + Kd * (leftError - leftPrev_error))/12);
    
    if (rightlateralMotorPower > double(10))
    
    {
      rightlateralMotorPower = 10.0;
      //return 0;
    }


    if (leftlateralMotorPower > double(10))
    {
      leftlateralMotorPower = 10.0;
      //return 0;
    }
    if (turnV > double(10))
    {
      turnV = 10.0;
    }

    leftside.spin(forward, leftlateralMotorPower + turnV, voltageUnits::volt);
    rightside.spin(forward, rightlateralMotorPower - turnV, voltageUnits::volt);
    //speed = Kp*error + Ki*integral + Kd * derivative;
    //return 0;
  } 

}
int whenStarted2()
{
  while (swtch)
  {
    actualturningV = actualturningKp*actualturningError +actualturningKi*actualturningIntegral + actualturningKd * (actualturningError - actualturningPrev_error);
    FrontRight.spin(forward, actualturningV, voltageUnits::volt);
    FrontLeft.spin(reverse, actualturningV, voltageUnits::volt);
    BackRight.spin(forward, actualturningV, voltageUnits::volt);
    BackLeft.spin(reverse, actualturningV, voltageUnits::volt);
  }
  return 0;
}

void onevent_getVar_5()
{
  while (swtch2)
  {
    getVar.broadcast();
    //Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    //printf(printToConsole_numberFormat(), static_cast<double>(turnError));
    //Controller1.Screen.print(" ");
    //Controller1.Screen.print(BackRight.position(degrees));
  }

}

// Event used to figure out when the PID has reached its destination
void reset()
{
  // Wait statement that waits for the error to be within a certian number before proceeding
  waitUntil((30 > leftError  && rightError < 30 && longLat ) || (turninput != 0.0 && turnError < 1 && Turn) );
  //Lets the driver know that the reset has begun
  Controller1.Screen.print("AAAAA");
  // disables all PID calculation
  swtch = false;
  //stops all motors to prevent any interference
  leftside.stop();
  rightside.stop();
  // Zeroing all PID values
  TV = 0.0;
  turninput = 0.0;
  actualturning = 0.0;
  actualturningV = 0.0;
  actualturningIntegral = 0.0;
  actualturningDerivative = 0.0;
  actualturningPrev_error = 0.0;
  actualturningError = 0.0;
  leftError = 0.0;
  leftIntegral = 0.0;
  leftDerivative = 0.0;
  leftPrev_error = 0.0;
  rightError = 0.0;
  rightIntegral = 0.0;
  rightDerivative = 0.0;
  rightPrev_error = 0.0;
  // Lets programmer know that the reset has ended
  Brain.Screen.print("MONKEY!");
  // Resets what movement the PID is doing and gives the all clear for the next PID movement to begin
  longLat = false;
  Turn = false;
  swtch = true;
  AC == true;
}


// Main auton code

int onauton_autonomous_0()
{
  // Zeros the motors.
  FrontLeft.setPosition(0, degrees);
  FrontRight.setPosition(0, degrees);
  BackLeft.setPosition(0, degrees);
  BackRight.setPosition(0, degrees);
  // First movement
  TV = moveLong(500.0);
  // Redundancy to make sure there is no interference
  swtch = true;
  getVar.broadcast();
  Brain.Screen.print("Running!");
  // broadcasting the reset
  reset12.broadcast();
  swtch = true;
  waitUntil(AC);
  TV = moveLong(500.0);
  reset12.broadcast();
  waitUntil(AC);
  turninput = 90;
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
  FrontLeft.setPosition(0, degrees);
  FrontRight.setPosition(0, degrees);
  BackLeft.setPosition(0, degrees);
  BackRight.setPosition(0, degrees);
  getVar(onevent_getVar_0);
  getVar(onevent_getVar_1);
  getVar(onevent_getVar_2);
  getVar(onevent_getVar_3);
  getVar(onevent_getVar_4);
  getVar(onevent_getVar_5);
  reset12(reset);
  vex::competition::bStopTasksBetweenModes = false;
  Competition.drivercontrol(VEXcode_driver_task);
  Competition.autonomous(VEXcode_auton_task);
  swtch = false;
  swtch2 = true;
  vex::task ws1(whenStarted2);


}