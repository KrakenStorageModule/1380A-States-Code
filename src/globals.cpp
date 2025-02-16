#include "globals.hpp"
#include "main.h"

// Controllers
pros::Controller controller(pros::E_CONTROLLER_MASTER); //physical controller 

// Motors
pros::Motor intakeHook(15, pros::MotorGears::blue); //hooks
pros::Motor intakeFront(14, pros::MotorGears::blue); //front intake
pros::Motor lbArm(-2, pros::MotorGears::green); //lady brown motor
pros::MotorGroup left_motor_group({-5, -4, -3}); //left drive motors
pros::MotorGroup right_motor_group({-4, 7, 8}); //right drive motors


// Sensors
pros::Rotation lbArmTrack(11); //lady brown rotation sensor
pros::Optical vision(20); //color sensor

// pistons
pros::adi::DigitalOut mogo('f'); //mogo clamp
pros::adi::DigitalOut rushArm('h'); //doinker

// toggles
bool mogoToggle = false; //toggle for mogo clamp
bool rushArmToggle = false; //toggle for doinker
bool colorSortToggle = false;

//stuff for colorsort
int color = 2; //color of alliance, defaulted to disable (0 = red, 1 = blue, 2 = disable)

// Controller Temp Display
double avgTempLeft = 0; //left drive half avg temp
double avgTempRight = 0; //right drive half avg temp
int avgTempTotal = 0; //avg temp of whole drivetrain
std::string tempReturn = " "; //conversion for display

// Chassis constructor
ez::Drive chassis(
    // These are your drive motors, the first motor is used for sensing!
    {-5, -4, -3},     // Left Chassis Ports (negative port will reverse it!)
    {-4, 7, 8},  // Right Chassis Ports (negative port will reverse it!)

    12,      // IMU Port
    3.25,  // Wheel Diameter (Remember, 4" wheels without screw holes are actually 4.125!)
    450);   // Wheel RPM = cartridge * (motor gear / wheel gear)

//drivetrain speeds
// These are out of 127
const int DRIVE_SPEED = 110;
const int TURN_SPEED = 90;
const int SWING_SPEED = 110;