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
motor BackRight = motor(PORT11, ratio18_1, true);

motor FrontRight = motor(PORT12, ratio18_1, false);

motor BackLeft = motor(PORT19, ratio18_1, false);

motor FrontLeft = motor(PORT18, ratio18_1, true);


inertial Inertial4 = inertial(PORT4);

// Z@

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


// Z@

#pragma region Tuning

//Lateral Tuning

double Kp = 0.375; // Constant for Proportion
double Ki = 0.0001; // Constant for Integral
double Kd = 0.0001;// Constant for Derivative

//Turn Tuning

double turnKp = 0.100; // Constant for Proportion
double turnKi = 0.0001; // Constant for Integral
double turnKd = 0.0001;// Constant for Derivative

//Turning Tuning
double actualturningKp = 0.0; // Constant for Proportion
double actualturningKi = 0.0; // Constant for Integral
double actualturningKd = 0.0; // Constant for Derivative


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

//The calculate event is used to update all the calculations and multiple instances of this event are used to allow for all the calculations to occur at one time.
event calculate = event();

//reset event is ued to reset all PID calculations.
event reset12 = event();

double leftlateralMotorPower = 0.0;
double rightlateralMotorPower = 0.0;

//PID variables attached to the turninput variable
double turnV = 0.0;
double turnError = .1; //error is the distance left from the current position to the TV
double turnIntegral = 0.0;
double turnDerivative = 0.0;
double turnPrev_error = 0.0;

//Average Right Motor Position
double AVGRMP = 0;

//Average Left Motor Position
double AVGLMP = 0;

// All clear boolean that is used to let the code knom that its good to run the next movement
bool AC = false;
//Variables to track what movement method is in use
bool longLat = false;
bool Turn = false;
bool TurnX = false;

//actual turning is for x-drive turning as turn causes side to side movement
double actualturningV = 0.0; //Voltage to give motors
double actualturningIntegral = 0.0;
double actualturningDerivative = 0.0;
double actualturningPrev_error = 0.0;
double actualturningError = 0.0;
double turningrot = 0.0;

// Z@

#pragma endregion Declarables 


int ondriver_drivercontrol_0() 
{
  FrontRight.setVelocity((((Controller1.Axis3.position()*-1) + Controller1.Axis1.position()) + Controller1.Axis4.position()) , percent);
  FrontLeft.setVelocity((((Controller1.Axis3.position()*-1) - Controller1.Axis1.position()) - Controller1.Axis4.position()), percent);
  BackRight.setVelocity((((Controller1.Axis3.position()*-1) - Controller1.Axis1.position()) + Controller1.Axis4.position()), percent);
  BackLeft.setVelocity((((Controller1.Axis3.position()*-1) + Controller1.Axis1.position()) - Controller1.Axis4.position()), percent);
  FrontRight.spin(forward);
  FrontLeft.spin(forward);
  BackRight.spin(forward);
  BackLeft.spin(forward);
  return 0;
}

void VEXcode_driver_task() {
  // Start the driver control tasks....
  vex::task drive0(ondriver_drivercontrol_0);
  while(Competition.isDriverControl() && Competition.isEnabled()) {this_thread::sleep_for(10);}
  drive0.stop();
  return;
}

// Funtion that sets the value and moves the robot along its longitundinal axis
double moveLong(double input)
{
  AC = false;
  TV = input/(3.14159 * 104.775) * 360; // Sets the target value, the equation converts mm to degrees becuase thats what all the math is based off of.
  longLat = true;
  //Enables the math loop
  swtch = true;
  //returns the updated target value
  return TV;
}

// the function that sets the distance to x drive side to side movement 
double turn(double input)
{
  AC = false;
  turninput = input;
  Turn = true;
  Controller1.rumble("...");
  Brain.Screen.print("Turn Code started");
  // Enables the math loop
  swtch = true;
  return 0;
}
double xTurn(double input)
{
  AC = false;
  actualturning = input;
  TurnX = true;
  //Enables the math loop
  swtch = true;
  return actualturning;
}
#pragma region SIDPID


// Left calculation
void leftLatCalc()
{
  AVGLMP = FrontLeft.position(degrees) + BackLeft.position(degrees)/2; // Average rotation for the left side motors
  (TV != 0.0) ? leftError = TV - AVGLMP : leftError = 0.0; // Shorthand if statement to see if the average rotation of both motors are equal to the target value, then sets the error to 0
  if (leftError == leftPrev_error)
  {
    leftIntegral = 0.0;
  }
  leftIntegral += leftError; // Updates the integral value
  leftPrev_error = leftError; // Updates the Previous error value
}

// Right Calculation

void rightLatCalc()
{
  AVGRMP = FrontRight.position(degrees) + BackRight.position(degrees)/2; // Average rotation for the right side motors.
  (TV != 0.0) ? rightError = TV - AVGRMP : rightError = 0.0;// Shorthand if statement to see if the average rotation of both motors are equal to the target value, then sets the error to 0
  if (rightError == rightPrev_error)
  {
    rightIntegral = 0.0;
    //AC = true; 
  }
  rightIntegral += rightError;// Updates the integral value
  rightPrev_error = rightError;// Updates the Previous error value
}
#pragma endregion SIDPID

//Turn Calculation, X-Drive side to side

void turnCalc()
{
  averagerot = (AVGRMP + AVGLMP/2); // Average rotation, X drive side to side distance
  (turninput != 0.0) ? turnError = (averagerot-turninput): turnError = 0.0;// Shorthand if statement to see if the average rotation of both motors are equal to the target value, then sets the error to 0
  if (turnError == turnPrev_error)// Resets integral value if the prev error and error both equal eachother
  {
    turnIntegral = 0.0;
    //AC = true;
  } 
  turnIntegral += turnError; // Updates the integral value 
  turnPrev_error = turnError; // Updates the Previous error value
  
  // Z@
}

//Calculation used for X-Drive turning.
void xTurnCalc()
{
  turningrot = fabs((FrontRight.position(degrees)) + fabs(BackRight.position(degrees))/7.44); // Uses the average of 2 motors to figure out where the x-drive is facing.
  (actualturning != 0.0) ? actualturning = (averagerot-actualturning): actualturningError = 0.0;// Shorthand if statement to see if the average rotation of both motors are equal to the target value, then sets the error to 0
  if (actualturningError == actualturningPrev_error)
  {
    actualturningIntegral = 0.0;
  }
  actualturningIntegral += actualturningError;// Updates the integral value 
  actualturningPrev_error = actualturningError;// Updates the Previous error value

}

//Final Calculation 
void theFinalCalcDown()
{
  while (swtch) 
  {

    turnV = turnKp*turnError + turnKi*turnIntegral + turnKd * (turnError - turnPrev_error);
    rightlateralMotorPower = ((Kp*rightError + Ki*rightIntegral + Kd * (rightError - rightPrev_error))/12);// Z@
    leftlateralMotorPower = ((Kp*leftError + Ki*leftIntegral + Kd * (leftError - leftPrev_error))/12);// Z@
    
    if (rightlateralMotorPower > double(10)) // Clamps the right motor power to 10 max
    
    {
      rightlateralMotorPower = 10.0;
    }

    if (rightlateralMotorPower < double(-10)) // Clamps the right motor power to 10 max
    {
      rightlateralMotorPower = -10.0;
    }


    if (leftlateralMotorPower > double(10)) // Clamps the left motor power to 10 max
    {
      leftlateralMotorPower = 10.0;
    }

    if (leftlateralMotorPower < double(-10)) // Clamps the left motor power to 10 max
    {
      leftlateralMotorPower = -10.0;
    }


    if (turnV > double(10)) // Clamps the turn motor power to 10 max
    {
      turnV = 10.0;
    }
    if (turnV < double(-10)) // Clamps the turn motor power to 10 max
    {
      turnV = -10.0;
    }
    FrontLeft.spin(forward, leftlateralMotorPower + turnV, voltageUnits::volt);
    BackLeft.spin(forward, leftlateralMotorPower + turnV, voltageUnits::volt);
    FrontRight.spin(forward, rightlateralMotorPower - turnV, voltageUnits::volt);
    BackRight.spin(forward, rightlateralMotorPower - turnV, voltageUnits::volt);
    //leftside.spin(forward, leftlateralMotorPower + turnV, voltageUnits::volt);// Z@
    //rightside.spin(forward, rightlateralMotorPower - turnV, voltageUnits::volt);
  } 
// THe calculation that allows for X-Drive turninc.
}
int actualTurningCalc()
{
  while (1==0)
  {
    actualturningV = actualturningKp*actualturningError +actualturningKi*actualturningIntegral + actualturningKd * (actualturningError - actualturningPrev_error);// Z@
    //FrontRight.spin(forward, actualturningV, voltageUnits::volt);
    //FrontLeft.spin(reverse, actualturningV, voltageUnits::volt);
    //BackRight.spin(reverse, actualturningV, voltageUnits::volt);
    //BackLeft.spin(forward, actualturningV, voltageUnits::volt);
  }
  return 0;
}

void debugThingy()
{
  while (swtch2)
  {
    calculate.broadcast();
    Controller1.Screen.clearScreen();
    Controller1.Screen.setCursor(1,1);
    //printf(printToConsole_numberFormat(), static_cast<double>(turnError));
    Controller1.Screen.print(" ");
    Controller1.Screen.print(turnV);
    Controller1.Screen.setCursor(2,2);
    Controller1.Screen.print(" ");
    Controller1.Screen.print(leftlateralMotorPower);
  }
}// Z@


void reset()// Event used to figure out when the PID has reached its destination
{
  //waitUntil((50 > fabs(leftError) && fabs(rightError) < 50 && longLat || (turnError < 1 && Turn))); 
  //Uses a wait statement that waits for the error (dependant upon what movement is occurring) within a small enough margin before resetting.
  waitUntil((1.2 > fabs(rightlateralMotorPower) && fabs(leftlateralMotorPower) < 1.2 && longLat) || (fabs(turnV) < 1 && Turn));
  //50 > fabs(leftError) && fabs(rightError) < 50 && longLat )); //|| (turninput != 0.0 && turnError < 1 && Turn)); 
  //Brain.Screen.print(leftError);
  //Brain.Screen.print(rightError);
  Brain.Screen.print(longLat);
  //Lets the driver know that the reset has begun
  Controller1.Screen.print("AAAAA");
  Controller1.rumble("-");

  //Disables the math loop so that the TV can be properly initalized
  swtch = false;

  //stops all motors to prevent any interference
  FrontRight.stop();
  FrontLeft.stop();
  BackRight.stop();
  BackLeft.stop();
  leftside.stop();
  rightside.stop();
  FrontLeft.setPosition(1, degrees);// Z@
  FrontRight.setPosition(1, degrees); // Z@
  BackLeft.setPosition(1, degrees); // Z@
  BackRight.setPosition(1, degrees);

  // Zeroing all PID values
  TV = 0.0;
  turninput = 0.0;
  averagerot = 0.0;
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
  rightlateralMotorPower = 0;
  leftlateralMotorPower = 0;

  turnV = 0;
  turnError = .1; 
  turnIntegral = 0.0;
  turnDerivative = 0.0;
  turnPrev_error = 0.0;
  averagerot = 0.0;

  // Resets what movement the PID is doing and gives the all clear for the next PID movement to begin
  longLat = false;
  Turn = false;
  
  // Lets programmer know that the reset has ended
  Brain.Screen.print("MONKEY!");
  // Lets the code know that the reset has ended
  AC = true;
}

// Main auton code

int onauton_autonomous_0()
{
  
  // Zeros the motors. To make sure they're at their starting position
  FrontLeft.setPosition(0, degrees);
  FrontRight.setPosition(0, degrees);
  BackLeft.setPosition(0, degrees);
  BackRight.setPosition(0, degrees);

  // Initial instruction
  TV = moveLong(-1000.0);

  // Redundancy to make sure there is no interference
  // MAKE SURE: This code NEEDS has to be after the intital instruction, Otherwise the target value will not initalize properly
  swtch = true; // Enabling math loop switch.
  calculate.broadcast(); //Starts the broadcast loop. details on page: (insert page here)
  Brain.Screen.print("Running!"); //Debugging: allows me to know that the auton code has ran.

  // broadcasting the reset
  //Needs to be placed after initial instruction, otherwise will reset the auton code instantly
  reset12.broadcast();

  // Redundancy to make sure there that the math loop switch has properly turned on
  swtch = true;

  //This line below waits for function "reset" to finish through the use of an all clear boolean (true or false)
  waitUntil(AC);
  turn(360);
  wait(200, msec);
  reset12.broadcast();
  waitUntil(AC);

  moveLong(1000.0);
  wait(200, msec);
  reset12.broadcast();
  waitUntil(AC);

  turn(-360.0);
  wait(200, msec);
  reset12.broadcast();
  waitUntil(AC);
  
  xTurn(50);


/*
  TV = moveLong(700.0);
  reset12.broadcast();
  waitUntil(AC);
*/
  return 0;
}
// Z@

void VEXcode_auton_task() {
  // Start the auton control tasks....
  vex::task auto0(onauton_autonomous_0);
  while(Competition.isAutonomous() && Competition.isEnabled()) {this_thread::sleep_for(10);}
  auto0.stop();
  return;

}
int main() {
  Competition.drivercontrol(VEXcode_driver_task);
  Competition.autonomous(VEXcode_auton_task);
  FrontLeft.setPosition(0, degrees);// Z@
  FrontRight.setPosition(0, degrees); // Z@
  BackLeft.setPosition(0, degrees); // Z@
  BackRight.setPosition(0, degrees);
  calculate(leftLatCalc);
  calculate(rightLatCalc);
  calculate(turnCalc);
  calculate(xTurnCalc);
  calculate(theFinalCalcDown);
  calculate(debugThingy);
  reset12(reset);
  vex::competition::bStopTasksBetweenModes = false;
  swtch = false;
  swtch2 = true;
  vex::task ws1(actualTurningCalc);

// Z@
}