#include "robot-config.hpp"
#include "pros/motors.hpp"
#include "constants.hpp"

pros::MotorGroup leftMotors({Ports::left1, Ports::left2, Ports::left3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({Ports::right1, Ports::right2, Ports::right3}, pros::MotorGearset::blue);

lemlib::Drivetrain drivetrain(
    &leftMotors, 
	&rightMotors,
	// TODO: change this
	Drivetrain::track_width, // track width in inches 
	Drivetrain::wheel_diameter, 
	Drivetrain::rpm, 
	Drivetrain::horizontal_drift
);

pros::IMU inertial(Ports::inertial);

lemlib::OdomSensors odom(
    nullptr, 
	nullptr, 
	nullptr,
	nullptr, 
	&inertial);

// lateral PID controller
lemlib::ControllerSettings lateral_controller(10, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              3, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in inches
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in inches
                                              500, // large error range timeout, in milliseconds
                                              20 // maximum acceleration (slew)
);

// angular PID controller
lemlib::ControllerSettings angular_controller(2, // proportional gain (kP)
                                              0, // integral gain (kI)
                                              10, // derivative gain (kD)
                                              3, // anti windup
                                              1, // small error range, in degrees
                                              100, // small error range timeout, in milliseconds
                                              3, // large error range, in degrees
                                              500, // large error range timeout, in milliseconds
                                              0 // maximum acceleration (slew)
);

lemlib::Chassis chassis(
	drivetrain, 
	lateral_controller, 
	angular_controller,
	odom
);

pros::Motor pre_roller(Ports::pre_roller);
pros::Motor elevator  (Ports::elevator);
pros::Motor shooter   (Ports::shooter);

pros::Controller controller(pros::E_CONTROLLER_MASTER);

State scoring_state(HOARD);

pros::adi::DigitalOut hook        (Ports::hook);
pros::adi::DigitalOut match_loader(Ports::match_loader);

pros::Distance front(Ports::front_distance);
pros::Distance left (Ports::left_distance);
pros::Distance right(Ports::right_distance);
pros::Distance back (Ports::back_distance);

