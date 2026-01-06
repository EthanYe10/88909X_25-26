#include "pose_correction.hpp"
#include "lemlib/pose.hpp"
#include "robot-config.hpp"
#include "constants.hpp"
#include <cmath>

bool PoseCorrector::correct_pose() {
    current_pose = chassis.getPose();
    corrected_pose.theta = current_pose.theta; // always trust imu for heading

    bool right = current_pose.x >= Field::field_x / 2.0;

    const double wallX = right ? Field::field_x : 0.0;
    const double wallY = 0.0;

    const double c = cos(current_pose.theta * degree_to_radian);
    const double s = sin(current_pose.theta * degree_to_radian);
    
    EyeInfo eyes[] = {
        {Sensors::left_distance_x_offset, Sensors::left_distance_y_offset, Sensors::left_distance_theta, read_left_distance()},
        {Sensors::right_distance_x_offset, Sensors::right_distance_y_offset, Sensors::right_distance_theta, read_right_distance()}, 
        {Sensors::front_distance_x_offset, Sensors::front_distance_y_offset, Sensors::front_distance_theta, read_front_distance()}, 
        {Sensors::back_distance_x_offset, Sensors::back_distance_y_offset, Sensors::back_distance_theta, read_back_distance()}
    };

    // these are the pose corrections
    double best_x_correction = 0.0; 
    double best_y_correction = 0.0;

    // temp variables to measure likelihood of a good correction
    double best_x_confidence = 0.0;
    double best_y_confidence = 0.0;

    for (auto& eye : eyes) {
        // eye global position
        double eX = current_pose.x + (eye.x_offset * c - eye.y_offset * s);
        double eY = current_pose.y + (eye.x_offset * s + eye.y_offset * c);

        // eye direction vectors
        double dX = cos((current_pose.theta+eye.theta) * degree_to_radian); // "directness" of the eye to x wall
        double dY = sin((current_pose.theta+eye.theta) * degree_to_radian); // "directness" of the eye to y wall

        // tX and tY are the distances along the direction vectors to the walls
        double tX = -1.0;
        double tY = -1.0;
        // x wall correction
        double x_confidence = right ? dX : -dX;
        if (x_confidence > 0.2 && fabs(dX) > 1e-2){
            tX = (wallX - eX) / dX;
        }
        // y wall correction
        double y_confidence = -dY;
        if (y_confidence > 0.2 && fabs(dY) > 1e-2){
            tY = (wallY - eY) / dY;
        }
        // determine which wall was hit (closest one if both positive, else only one positive)
        char hit = 0;
        if (tX >= 0.0 && tY >= 0.0) { hit = (tX < tY) ? 'X' : 'Y'; } 
        else if (tX >= 0.0) { hit = 'X'; } 
        else if (tY >= 0.0) { hit = 'Y'; }
        else continue; // didn't hit wall

        // calculate candidate corrected positions
        if (hit == 'X' && eye.reading >= 0.0) {
            double x_candidate = wallX - eye.reading * dX - (eye.x_offset * c - eye.y_offset * s);
            if (x_confidence > best_x_confidence) {
                best_x_confidence = x_confidence;
                best_x_correction = x_candidate;
            }
        } else if (hit == 'Y' && eye.reading >= 0.0) {
            double y_candidate = wallY - eye.reading * dY - (eye.x_offset * s + eye.y_offset * c);
            if (y_confidence > best_y_confidence) {
                best_y_confidence = y_confidence;
                best_y_correction = y_candidate;
            }
        }
    }
    // apply best corrections if confident enough
    if (best_x_confidence > pose_correction_min_confidence && best_y_confidence > pose_correction_min_confidence) {
        corrected_pose.x = best_x_correction;
        corrected_pose.y = best_y_correction;
        return true;
    }
    return false;
}