#include "robot-config.hpp"
#include "robot.hpp"

constexpr double rotation_coef = 0.6;
constexpr int deadband = 15;

inline int apply_deadband(int value) {
    return (std::abs(value) < deadband) ? 0 : value;
}

void run_intake() {
    switch (scoring_state) {
        case (HOARD):
            pre_roller.move(127);
            elevator.move(127);
            shooter.brake();
            break;
        case(TOP):
            pre_roller.move(127);
            elevator.move(127);
            shooter.move(127);
            break;
        case (MIDDLE):
            pre_roller.move(127);
            elevator.move(127);
            shooter.move(-127);
            break;
    }
}

void reverse_intake() {
    pre_roller.move(-127);
    elevator.move(-127);
    shooter.brake();
}

void brake_intake() {
    pre_roller.brake();
    elevator.brake();
    shooter.brake();
}

// both adis start at false, which means hook is down and match loader is retracted
// when true, hook is up and match loader is deployed
bool hook_status = false; 
bool matchloader_status = false;

inline void flip_hook() {
    hook.set_value(!hook_status);
    hook_status ^= true;
}

inline void flip_match_loader() {
    match_loader.set_value(!matchloader_status);
    matchloader_status ^= true;
}

inline void raise_hook() {
    hook.set_value(true);
    hook_status = true;
}

inline void lower_hook() {
    hook.set_value(false);
    hook_status = false;
}

inline void deploy_match_loader() {
    match_loader.set_value(true);
    matchloader_status = true;
}

inline void retract_match_loader() {
    match_loader.set_value(false);
    matchloader_status = false;
}

void run_drivetrain() { // simple two-stick arcade with deadband and rotation scaling
    int straight = apply_deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
    int rotation = apply_deadband(controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));

    if (rotation == 0 && straight != 0) {
        rightMotors.move(straight);
        leftMotors.move(straight);
    } else if (rotation != 0) {
        rightMotors.move(rotation_coef * (straight - rotation));
        leftMotors.move(rotation_coef * (straight + rotation));
    } else {
        rightMotors.move(0);
        leftMotors.move(0);
    }
}

void driver_intake() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
        run_intake();
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
        reverse_intake();
    } else {
        brake_intake();
    }
}

void switch_intake_mode() {
    if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
        scoring_state = HOARD;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
        scoring_state = TOP;
    } else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
        scoring_state = MIDDLE;
    }
}

void get_pneumatic_inputs() {
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
        flip_match_loader();
    } 
    if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
        flip_hook();
    }
}

void run_driver() {
    run_drivetrain();
    driver_intake();
    switch_intake_mode();
    get_pneumatic_inputs();
}
