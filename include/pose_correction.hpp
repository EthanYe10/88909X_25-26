#include "lemlib/pose.hpp"
#include "pros/distance.hpp"
#include "robot-config.hpp"
#include "constants.hpp"

/*returns the reading of a distance sensor in inches, or -1 if invalid (outside sensor range/too low to be accurate)*/
static double read_distance(pros::Distance& sensor) {
    const double reading = sensor.get() * mm_to_in;
    if (reading < PoseCorrection::distance_deadzone_lower || reading > PoseCorrection::distance_deadzone_upper) {
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

static double angle_diff(double a, double b) {
    double diff = a-b;
    while (diff > 180.0) diff -= 360.0;
    while (diff < -180.0) diff += 360.0;
    return diff;
}

class PoseCorrector {
// IMPORTANT:
// this uses kalman filtering. Kalman filter takes a measurement and fuses it with a prediction (both are noisy) to get better estimate of actual state
// in this case odometry is the prediction, and distance sensor resets are the measurements
public: 
    // lemlib odometry info
    double Px, Py; // aggregate prediction uncertainty

    double speed_filtered = 0.0;
    double yaw_rate_filtered = 0.0;
    double acceleration_filtered = 0.0;
    double previous_speed = 0.0;

    bool turning = false;
    bool accelerating = false;

    bool motion_tracking_initialized = false;

    struct EyeInfo {
        double x_offset;
        double y_offset;
        double theta; // in degrees
        double reading;
    };

    struct DistanceMeasurement {
        double x, y;
        double Rx, Ry; // aggregate measurement uncertainty
    } measurement; // updated every control loop if correct_pose() returns true

    lemlib::Pose prediction; // current position from odometry
    lemlib::Pose previous_prediction; // used to determine acceleration, turning, etc for noise estimation
    
    bool correct_pose(); // calculate the distance sensor pose
    void calculate_odom_uncertainty(); // calculates Px, Py based on robot motion
    bool corrected_pose_is_valid() const; // uses mahalanobis distance to check validity (i lwk don't even know what this means)
    lemlib::Pose fuse_pose(); // fuses the new corrected pose with current pose using linear kalman filter
    void update(); // does stuff (yeah very cool i know)

    PoseCorrector() 
    : Px(0.0), Py(0.0)
    , motion_tracking_initialized(false)
    , prediction(lemlib::Pose{0, 0, 0})
    , previous_prediction(lemlib::Pose{0, 0, 0}) {}
};