#pragma once

#include "lemlib/chassis/trackingWheel.hpp"

// user set constants
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


