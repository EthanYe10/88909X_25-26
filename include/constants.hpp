#pragma once

#include "lemlib/chassis/trackingWheel.hpp"
#include <cmath>

constexpr double mm_to_in = 0.0393701;
constexpr double degree_to_radian = M_PI / 180.0;
constexpr double radian_to_degree = 180.0 / M_PI;

constexpr double dt = 0.01; // control loop

namespace Ports {
    constexpr int left1 = -3;
    constexpr int left2 = -14;
    constexpr int left3 = -12;

    constexpr int right1 = 10;
    constexpr int right2 = 17;
    constexpr int right3 = 19;

    constexpr int pre_roller = 4;
    constexpr int elevator = 11;
    constexpr int shooter = 13;

    constexpr int inertial = 11;

    constexpr int left_distance = 20;
    constexpr int right_distance = 5;
    constexpr int front_distance = 9;
    constexpr int back_distance = 12;

    constexpr char hook = 'G';
    constexpr char match_loader = 'F';

}

namespace Drivetrain {
    constexpr double wheel_diameter = lemlib::Omniwheel::NEW_325; // in inches
    constexpr double track_width = 11.5; // in inches
    constexpr double wheel_base = 11.5; // in inches
    constexpr float rpm = 450.0;
    constexpr float horizontal_drift = 2.0;
}

namespace Gains {
    constexpr float lateral_kP = 10.0;
    constexpr float lateral_kI = 0.0;
    constexpr float lateral_kD = 3.0;

    constexpr float angular_kP = 2.0;
    constexpr float angular_kI = 0.0;
    constexpr float angular_kD = 10.0;
}

namespace Sensors {
    constexpr double left_distance_x_offset = 2.75;
    constexpr double left_distance_y_offset = 5.5;
    constexpr double left_distance_theta = 90; // deg

    constexpr double right_distance_x_offset = 3.25;
    constexpr double right_distance_y_offset = -4.625;
    constexpr double right_distance_theta = -90; // deg

    constexpr double front_distance_x_offset = 4.625;
    constexpr double front_distance_y_offset = 5.25;
    constexpr double front_distance_theta = 0; // deg

    constexpr double back_distance_x_offset = -6.0;
    constexpr double back_distance_y_offset = 6.0;
    constexpr double back_distance_theta = 180; // deg
}

namespace Field {
    constexpr double tile_length = 24.0; // in inches
    constexpr double half_tile = tile_length / 2.0;
    constexpr double goal_height = 40.0; // in inches

    // y is half of x, because the two sides (blue and red) are identical, so we do not have to differentiate between them
    constexpr double field_x = 144.0; // in inches
    constexpr double field_y = 72.0;  // in inches
}

namespace PoseCorrection {
    constexpr double Qbase = 0.5; // base odom noise (TUNE THIS)
    constexpr double QturnMultiplier = 2.0; // multiplier for Q when turning (TUNE THIS)
    constexpr double QaccelMultiplier = 1.5; // multiplier for Q when accelerating (TUNE THIS)

    constexpr double distance_r_base = 15 * mm_to_in; // base distance sensor noise in inches (15mm) 
    constexpr double distance_r_factor = 0.05; // factor of reading to add to distance sensor noise (5%)

    constexpr double distance_deadzone_lower = 4.0;  // v5rc distance sensor min range to preserve accuracy
    constexpr double distance_deadzone_upper = 78.74016; // v5rc distance sensor max range in inches
    constexpr double pose_correction_min_confidence = 0.4; // can max be 66 degrees off wall direction
    constexpr double distance_max_diff_from_expected = 2.0; // in inches

    constexpr double mahalanobis_distance_threshold_squared = 9.0; // TUNE THIS
    
    // TUNE THESE
    constexpr double min_turn_speed_threshold = 2.0;
    constexpr double min_accel_threshold = 5.0; 

    // THIS TOO
    constexpr double max_pose_correction_per_update = 1.0; // in inches
}
