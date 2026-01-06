#include "lemlib/pose.hpp"
#include "pros/distance.hpp"
#include "robot-config.hpp"
#include "constants.hpp"

/*returns the average of 3 readings from the distance sensor in inches, */
static double read_distance(pros::Distance& sensor) {
    const double reading = sensor.get() * mm_to_in;
    if (reading < distance_deadzone_lower || reading > distance_deadzone_upper) {
        return -1.0; // invalid reading
    }
    return reading;
}

static double read_left_distance() {
    return read_distance(left);
}

static double read_right_distance() {
    return read_distance(right);
}

static double read_front_distance() {
    return read_distance(front);
}

static double read_back_distance() {
    return read_distance(back);
}

class PoseCorrector {
public: 
    struct EyeInfo {
        double x_offset;
        double y_offset;
        double theta; // in degrees
        double reading;
    };

    lemlib::Pose corrected_pose{0, 0, 0}; // future position to be filtered and applied
    lemlib::Pose current_pose; // current position from odometry
    lemlib::Pose previous_corrected_pose; // last applied corrected position
    bool correct_pose();
    // helpers
    bool correct_pose_left();
    bool correct_pose_right();

};