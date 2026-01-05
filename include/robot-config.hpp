#pragma once

#include "api.h" // IWYU pragma: keep
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/adi.hpp"

extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern lemlib::Drivetrain drivetrain;

extern pros::IMU inertial;

// TODO: update ports   

extern lemlib::OdomSensors odom;
// lateral PID controller
extern lemlib::ControllerSettings lateral_controller;

// angular PID controller
extern lemlib::ControllerSettings angular_controller;
extern lemlib::Chassis chassis;

extern pros::Controller controller;

extern pros::Motor pre_roller;
extern pros::Motor elevator;
extern pros::Motor shooter;

enum State {
    HOARD, 
    TOP, 
    MIDDLE
};

extern State scoring_state;

extern pros::adi::DigitalOut hook;
extern pros::adi::DigitalOut match_loader;