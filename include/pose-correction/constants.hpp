#pragma once

#include <cmath>

namespace Field {
    constexpr double tile_length = 24.0; // in inches
    constexpr double half_tile = tile_length / 2.0;
    constexpr double goal_height = 40.0; // in inches

    // y is half of x, because the two sides (blue and red) are identical, so we do not have to differentiate between them
    constexpr double field_x = 144.0; // in inches
    constexpr double field_y = 72.0;  // in inches
}

constexpr double mm_to_in = 0.0393701;
constexpr double degree_to_radian = M_PI / 180.0;
constexpr double radian_to_degree = 180.0 / M_PI;

constexpr double dt = 0.01; // control loop
