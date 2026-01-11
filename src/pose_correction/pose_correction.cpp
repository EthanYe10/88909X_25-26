#include "pose-correction/pose_correction.hpp"
#include "lemlib/chassis/odom.hpp"
#include "lemlib/pose.hpp"
#include "pose-correction/constants.hpp"
#include "pose-correction/tuning.hpp"
#include <cmath>

bool PoseCorrector::correct_pose() {
    bool right = prediction.x >= Field::field_x / 2.0;

    const double wallX = right ? Field::field_x : 0.0;
    const double wallY = 0.0;

    const double c = cos(prediction.theta * degree_to_radian);
    const double s = sin(prediction.theta * degree_to_radian);
    
    EyeInfo eyes[] = {
        {distance_sensors->left_offset.x_offset, distance_sensors->left_offset.y_offset, distance_sensors->left_offset.theta_offset , distance_sensors->getLeftDistance()},
        {distance_sensors->right_offset.x_offset, distance_sensors->right_offset.y_offset, distance_sensors->right_offset.theta_offset, distance_sensors->getRightDistance()},
        {distance_sensors->front_offset.x_offset, distance_sensors->front_offset.y_offset, distance_sensors->back_offset.theta_offset, distance_sensors->getFrontDistance()},
        {distance_sensors->back_offset.x_offset, distance_sensors->back_offset.y_offset, distance_sensors->back_offset.theta_offset, distance_sensors->getBackDistance()}
    };

    // these are the pose corrections
    double best_x_correction = 0.0; 
    double best_y_correction = 0.0;

    // temp variables to measure likelihood of a good correction
    double best_x_confidence = 0.0;
    double best_y_confidence = 0.0;

    for (auto& eye : eyes) {
        // eye global position
        const double eX = prediction.x + (eye.x_offset * c - eye.y_offset * s);
        const double eY = prediction.y + (eye.x_offset * s + eye.y_offset * c);

        // eye direction vectors
        const double dX = cos((prediction.theta+eye.theta) * degree_to_radian); // "directness" of the eye to x wall
        const double dY = sin((prediction.theta+eye.theta) * degree_to_radian); // "directness" of the eye to y wall

        // tX and tY are the distances along the direction vectors to the walls, essentially what the distance sensor should read
        // they start at an invalid value, and are only set if the eye is facing the wall
        double tX = -1.0;
        double tY = -1.0;
        // x wall correction
        const double x_confidence = right ? dX : -dX;
        if (x_confidence > 0.2 && fabs(dX) > 1e-2){
            tX = (wallX - eX) / dX;
        }
        // y wall correction
        const double y_confidence = -dY;
        if (y_confidence > 0.2 && fabs(dY) > 1e-2){
            tY = (wallY - eY) / dY;
        }
        // determine which wall was hit (closest one if both positive, else only one positive)
        // this works because if one sensor "sees" only 1 wall, then that is the only possible wall it could have hit
        // if it "sees" both walls, then it must have hit the closer one
        // if the difference between expected and actual reading is too large, skip this sensor
        char hit = 0;
        if (tX >= 0.0 && tY >= 0.0) { hit = (tX < tY) ? 'X' : 'Y'; } 
        else if (tX >= 0.0) hit = 'X';
        else if (tY >= 0.0) hit = 'Y'; 
        else continue; // didn't hit wall, skip this sensor
        
        // check difference between expected and actual reading
        const double expected_reading = (hit == 'X') ? tX : tY;
        if (fabs(expected_reading - eye.reading) > (PoseCorrection::distance_max_diff_from_expected + 0.05 * expected_reading)) continue; 

        // calculate candidate corrected positions
        if (hit == 'X' && eye.reading >= 0.0) {
            const double x_candidate = wallX - eye.reading * dX - (eye.x_offset * c - eye.y_offset * s);
            if (x_confidence > best_x_confidence) {
                best_x_confidence = x_confidence;
                best_x_correction = x_candidate;
                // calculate sensor uncertainty for x
                const double sigma_x = PoseCorrection::distance_r_base + PoseCorrection::distance_r_factor * eye.reading; // 15mm base error + 5% of reading, from VEX official specs
                measurement.Rx = sigma_x * sigma_x; // variance
            }
        } else if (hit == 'Y' && eye.reading >= 0.0) {
            const double y_candidate = wallY - eye.reading * dY - (eye.x_offset * s + eye.y_offset * c);
            if (y_confidence > best_y_confidence) {
                best_y_confidence = y_confidence;
                best_y_correction = y_candidate;
                const double sigma_y = PoseCorrection::distance_r_base + PoseCorrection::distance_r_factor * eye.reading;
                measurement.Ry = sigma_y * sigma_y;
            }
        }
    }

    // apply best corrections if confident enough
    if (best_x_confidence > PoseCorrection::pose_correction_min_confidence && best_y_confidence > PoseCorrection::pose_correction_min_confidence) {
        measurement.x = best_x_correction;
        measurement.y = best_y_correction;
        return true;
    }
    return false;
}

void PoseCorrector::calculate_odom_uncertainty() {

    /* step 1: calculate the robot's motion */
    constexpr double smoothing_factor = 0.2;
    // calculate whether robot is turning or accelerating, for odom noise estimation
    if (!motion_tracking_initialized) {
        previous_prediction = prediction;
        motion_tracking_initialized = true;
    }

    double dx = prediction.x - previous_prediction.x;
    double dy = prediction.y - previous_prediction.y;
    double dtheta = angle_diff(prediction.theta, previous_prediction.theta);

    double yaw_rate = fabs(dtheta) / dt; // degrees per second
    double speed = sqrt(dx*dx + dy*dy) / dt; // inches per second

    yaw_rate_filtered = (1.0 - smoothing_factor) * yaw_rate_filtered + smoothing_factor * yaw_rate;
    speed_filtered = (1.0 - smoothing_factor) * speed_filtered + smoothing_factor * speed;

    double acceleration = (speed_filtered - previous_speed) / dt;
    acceleration_filtered = (1.0 - smoothing_factor) * acceleration_filtered + smoothing_factor * acceleration;

    previous_speed = speed_filtered;
    previous_prediction = prediction;

    /* step 2: determine whether robot is turning or accelerating */
    turning = yaw_rate_filtered > PoseCorrection::min_turn_speed_threshold; // deg/s threshold
    accelerating = acceleration_filtered > PoseCorrection::min_accel_threshold; // in/s^2 threshold

    /* step 3: calculate Px, Py based on motion */
    double Q = PoseCorrection::Qbase;
    if (turning) {
        Q *= PoseCorrection::QturnMultiplier; 
    }
    if (accelerating) {
        Q *= PoseCorrection::QaccelMultiplier;
    }
    Px += Q;
    Py += Q;
}

bool PoseCorrector::corrected_pose_is_valid() const {
    // calculate mahalanobis distance
    double dx = measurement.x - prediction.x;
    double dy = measurement.y - prediction.y;

    double d2 = 
        (dx * dx) / (Px + measurement.Rx) + 
        (dy * dy) / (Py + measurement.Ry);
    return d2 < PoseCorrection::mahalanobis_distance_threshold_squared; 
}

lemlib::Pose PoseCorrector::fuse_pose() {
    lemlib::Pose fused_pose = prediction;
    fused_pose.theta = prediction.theta;

    double dx = measurement.x - prediction.x;
    double dy = measurement.y - prediction.y;

    // innovation variance
    double Sx = Px + measurement.Rx;;
    double Sy = Py + measurement.Ry;

    // Kalman gain
    double Kx = Px / Sx;
    double Ky = Py / Sy;

    // update pose
    fused_pose.x += Kx * dx;
    fused_pose.y += Ky * dy;

    // update uncertainties
    Px = (1.0 - Kx) * Px;;
    Py = (1.0 - Ky) * Py;

    return fused_pose;
}

void PoseCorrector::update() {
    if (distance_sensors == nullptr) return; // safety check

    // first get odom measurement 
    prediction = lemlib::getPose();
    // calculate odom uncertainty
    calculate_odom_uncertainty();

    // get corrected pose from distance sensors
    if (correct_pose()) {
        // check if corrected pose is valid
        if (corrected_pose_is_valid()) {
            // fuse poses
            const lemlib::Pose fused = fuse_pose();

            // calculate change in pose 
            const double dx = fused.x - prediction.x;
            const double dy = fused.y - prediction.y;
            if (fabs(dx) < PoseCorrection::max_pose_correction_per_update || fabs(dy) < PoseCorrection::max_pose_correction_per_update) { // max 1 inch correction to prevent large jumps 
                lemlib::setPose(fused);
            }
        }
    }

    // HOLY CONDITION STACKING
    // gg ez chat is the goat
}