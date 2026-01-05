#include "lemlib/chassis/trackingWheel.hpp"

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