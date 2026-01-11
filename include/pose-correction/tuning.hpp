#pragma once

#include "pose-correction/constants.hpp"

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

