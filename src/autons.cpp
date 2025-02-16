#include "driverControl.hpp"
#include "main.h"

/////
// For installation, upgrading, documentations, and tutorials, check out our website!
// https://ez-robotics.github.io/EZ-Template/
/////
//these are the subsystem functions
//autonMogo(); => call this to change the state of the mogo clamp
//autoDoinker(); => call this to change the state of the doinker

//intake(); => call this to intake indefinitely until another intake function is called
//outtake(); => call this to outtake indefinitely until another intake function is called
//Intakekill(); => call this to stop the intake indefinitely until another intake function is called

//nextState(); => call this to move the state of the lady brown forward
//backState(); => call this to move the state of the lady brown back
//untipState(); => call this to move the lady brown to the untip position, call it again to return to stow
//tippingState(); => call this to move the lady brown to the tipping position, call it again to return to stow

//when the auton is red, make color = 0;
//when the auton is blue, make color = 1;
//when the auton should not run colorsort, delete the pros::Task racism(autonColorsort); line




// 0 = off, 1 = intake, 2 = outtake
int intakeState = 0;
// Auton Subsystem Functions

// THESE ARE INTAKE FUNCTIONS
// TO RUN FRONT INTAKE ALONE =>
// intakeFront.move_voltage(-12000); (- for intake, + for outtake)
// leave intakestate alone => it is for colorsort (it lets colorsort return to intaking after it throws a ring)
// call this to set the full intake to intake indefinitely until another intake function is called
void intake() {
  intakeHook.move_voltage(-1200000);
  intakeFront.move_voltage(-1200000);
  intakeState = 1;
}

// call this to set the full intake to outtake indefinitely until another intake function is called
void outtake() {
  intakeHook.move_voltage(1200000);
  intakeFront.move_voltage(1200000);
  intakeState = 2;
}

// call this to stop the full intake indefinitely until another intake function is called
void Intakekill() {
  intakeHook.move_voltage(0);
  intakeFront.move_voltage(0);
  intakeState = 0;
}

// These are effectively toggles. call it to change the state of the pistons

// mogo
void autonMogo() {
  mogoToggle = !mogoToggle;
  mogo.set_value(mogoToggle);
}

// doinker
void autoDoinker() {
  rushArmToggle = !rushArmToggle;
  rushArm.set_value(rushArmToggle);
}

// auton colorsort
// 0 is red, 1 is blue, 2 is disable
// color controls colorsort
float currVoltage = 0;  // what the voltage of the intake will return to after colorsorting
void autonColorsort() {
  while (true) {
    // Correctly update the global currVoltage
    if (intakeState == 1) {
      currVoltage = -12000;  // Reverse intake
    } else if (intakeState == 2) {
      currVoltage = 12000;  // Forward intake
    } else {
      currVoltage = 0;  // Stop intake
    }

    // RED alliance mode: EJECT BLUE rings
    if (color == 0 && (vision.get_hue() <= 270 && vision.get_hue() >= 200) &&
        vision.get_proximity() > 100) {
      pros::delay(70);                       // Delay to allow ring to reach apex
      intakeHook.move_voltage(0);            // Eject
      pros::delay(70);                       // allows ring to fly out fully
      intakeHook.move_voltage(currVoltage);  // Resume intake operation
    }

    // BLUE alliance mode: EJECT RED rings
    else if (color == 1 &&
             (vision.get_hue() >= 300 && vision.get_hue() <= 400) &&
             vision.get_proximity() > 100) {
      pros::delay(70);                       // Delay to allow ring to reach apex
      intakeHook.move_voltage(0);            // Eject
      pros::delay(70);                       // allows ring to fly out fully
      intakeHook.move_voltage(currVoltage);  // Resume intake operation
    }

    pros::delay(5);
  }
}


// . . .
// Make your own autonomous functions here!
// . . .

void ringRushRed() {
  // just testing motion, not an actual auto dw
  chassis.odom_xyt_set(0_in, 0_in, 0_deg);
  chassis.pid_odom_set({{0_in, 2_in}, fwd, 110});
  // autoDoinker();
  intakeFront.move_voltage(-12000);
  chassis.pid_wait();
  intakeFront.move_voltage(0);
  chassis.pid_turn_set(90_deg, TURN_SPEED);
  intake();
  chassis.pid_wait();
  Intakekill();

}