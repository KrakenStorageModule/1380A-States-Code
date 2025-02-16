#pragma once

#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/misc.hpp"
#include "EZ-Template/api.hpp"

// Controllers
extern pros::Controller controller;

// Motors
extern pros::Motor intakeHook;
extern pros::Motor intakeFront;
extern pros::Motor lbArm;
extern pros::MotorGroup left_motor_group;
extern pros::MotorGroup right_motor_group;

// Sensors
extern pros::Rotation lbArmTrack;
extern pros::Optical vision;

// Pistons
extern pros::adi::DigitalOut mogo;
extern pros::adi::DigitalOut rushArm;

// Toggles
extern bool mogoToggle;
extern bool rushArmToggle;

// Controller Temp Display
extern double avgTempLeft;
extern double avgTempRight;
extern int avgTempTotal;
extern std::string tempReturn;

// Chassis constructor
extern ez::Drive chassis;
extern ez::tracking_wheel horiz_tracker;

// Temperature Controller Display
extern void controllerHud();

// LADY BROWN CODE
extern const int numStates;
extern int states[];
extern int currState;
extern int target;
extern double kP;
extern double kI;
extern double kD;
extern double error;
extern double prevError;
extern double armIntegral;
extern double derivative;
extern double output;
extern bool tippingVar;
extern bool untipVar;
extern int color;
void backState();
void nextState();
void untipState();
void tippingState();
void armDriver();

// PISTON CODE
void pneumaticDriverControl();

// Intake control
void intakeDriver();
void intakeControl();