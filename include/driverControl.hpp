#pragma once

#include "pros/adi.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "pros/motors.hpp"
#include "pros/motor_group.hpp"
#include "pros/misc.hpp"
#include "EZ-Template/api.hpp"
#include "globals.hpp"

// Temperature Controller Display
extern void controllerHud();

// PISTON CODE
void pneumaticDriverControl();

// Intake control
void intakeDriver();
void intakeControl();