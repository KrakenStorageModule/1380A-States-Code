#include "EZ-Template\api.hpp"
#include "autons.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/adi.hpp"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rotation.hpp"
#include "driverControl.hpp"
// Temperature Controller Display
void controllerHud() {
  while (true) {
    // Averaging each dt half
    
    avgTempLeft = (left_motor_group.get_temperature(0) +
                   left_motor_group.get_temperature(1) +
                   left_motor_group.get_temperature(2)) /
                  3;

    avgTempRight = (right_motor_group.get_temperature(0) +
                    right_motor_group.get_temperature(1) +
                    right_motor_group.get_temperature(2)) /
                   3;
    // Convert to F while averaging both sides
    avgTempTotal = int((((avgTempLeft + avgTempRight) * 1.8) / 2) + 32);
    // Convert to string and display
    tempReturn = std::to_string(avgTempTotal);
    controller.set_text(0, 0, "DT: " + tempReturn + " ");
    pros::delay(50);
  }
}

// LADY BROWN CODE
// U change the position of each state by changing the number in states[numstates] = {};
const int numStates = 3;
// make sure these are in centidegrees (1 degree = 100 centidegrees)
int states[numStates] = {30000, 26800, 14000};  // Bigger number = closer to stowed
int currState = 0;                              // current state (index of the array)
int target = 30000;                             // this must be the same as whatever the stow state is!
// PID constants => should be left alone
double kP = 2;    //"Gas" pedal
double kI = 0.0;  // no touch
double kD = 0;    // Adds more "slow-down" towards the end of a motion

// PID variables
double error = 0;        // difference between target and current position
double prevError = 0;    // previous error
double armIntegral = 0;  // sum of all errors
double derivative = 0;   // difference between current and previous error
double output = 0;       // final output to the lady brown
bool tippingVar = true;  // toggle for tipping
bool untipVar = true;    // toggle for untipping

// moves the state back 1
void backState() {
  currState = (currState - 1 + numStates) % numStates;
  target = states[currState];
  // setting them to true "resets" the toggles
  untipVar = true;
  tippingVar = true;
}

// moves the state forward 1
void nextState() {
  currState = (currState + 1) % numStates;
  target = states[currState];
  // setting them to true "resets" the toggles
  untipVar = true;
  tippingVar = true;
}

// short lady brown functions
// lowest position -> used for untipping
void untipState() {
  // this works as a toggle for the untip
  untipVar = !untipVar;
  // resets the tipping toggle
  tippingVar = true;
  if (untipVar == false) {
    target = 6500;
  } else {
    // makes the arm go to stow after hitting the untip button again
    target = states[0];
  }
}

// slightly lower than scoring -> tipping mogos
void tippingState() {
  // this works as a toggle for the tipping
  tippingVar = !tippingVar;
  //"resets" the untip toggle
  untipVar = true;
  if (tippingVar == false) {
    target = 10000;
  } else {
    // makes the arm go to stow after hitting the tipping button again
    target = states[0];
  }
}

// Actual logic for lady brown
void armDriver() {
  while (true) {
    // basically the button logic
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      nextState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
      backState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
      tippingState();
    } else if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
      untipState();
    }

    // if its in stow it coasts the motor to let the ring go in further
    // lowk this does nothing, but it sounds cool during interview!
    if (currState == 0) {
      lbArm.set_brake_mode(pros::MotorBrake::coast);
    } else {
      lbArm.set_brake_mode(pros::MotorBrake::hold);
    }
    // PID calculations
    error = target - lbArmTrack.get_position();                      // difference between target and current position
    armIntegral += error;                                            // sum of all errors
    derivative = error - prevError;                                  // difference between current and previous error
    output = (kP * error) + (kI * armIntegral) + (kD * derivative);  // final output to the lady brown

    lbArm.move_voltage(output);  // actually move the arm

    prevError = error;
    pros::delay(20);
  }
}

// PISTON CODE
void pneumaticDriverControl() {
  // mogo
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
    mogoToggle = !mogoToggle;
    mogo.set_value(mogoToggle);
  }
  // //ben pibotics arm
  if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
    rushArmToggle = !rushArmToggle;
    rushArm.set_value(rushArmToggle);
  }
}
// part 1 of intake control.
// this is called whenever colorsort is not throwing rings
void intakeDriver() {
  // intake
  if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
    intakeFront.move_voltage(-12000);
    intakeHook.move_voltage(-12000);
    // outtake
  } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
    intakeFront.move_voltage(12000);
    intakeHook.move_voltage(12000);
    // stop
  } else {
    intakeFront.move_voltage(0);
    intakeHook.move_voltage(0);
  }
}

/**
 * @brief A function that uses the vision sensor to sort rings.
 *
 * Compares the color of the ring detected by the vision sensor to the color of
 * the ring in the color variable. If the colors are different, the function
 * kicks out the wrong color ring by spinning the intake hook motor forward.
 */

void intakeControl() {
  while (true) {
    // checks to see if peter turned colorsort off or not
    // setting colorsortToggle to true in opcontrol will disable colorsort permanantly
    if (!colorSortToggle) {
      // RED
      // Senses for blue rings to eject
      if (color == 0 && (vision.get_hue() <= 270 && vision.get_hue() >= 200) && vision.get_proximity() > 100) {
        pros::delay(70);  // gives ring time to reach apex

        intakeHook.move_voltage(0);  // stops intake
        pros::delay(70);             // ensures the ring has time to fly
      }
      //  BLUE
      // Senses for red rings to eject
      else if (color == 1 &&
               (vision.get_hue() >= 300 && vision.get_hue() <= 400) &&
               vision.get_proximity() > 100) {
        pros::delay(70);             // gives ring time to reach apex
        intakeHook.move_voltage(0);  // stops intake
        pros::delay(70);             // ensures the ring has time to fly

      } else {
        // when colorsort is turned on but not throwing rings
        intakeDriver();  // Normal intake control
      }
    } else {
      // when colorsort is turned off
      intakeDriver();
    }
    // threading protection so the brain doesnt die
    pros::delay(20);
  }
}
