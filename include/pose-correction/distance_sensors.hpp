#pragma once

#include "pros/distance.hpp"
#include "pose-correction/constants.hpp"

class DistanceSensors {
public:
    struct Offsets{
        double x_offset;
        double y_offset;
        double theta_offset;
    };

    DistanceSensors(int left_port, int right_port, int front_port, int back_port,
                    Offsets left_offset, Offsets right_offset,
                    Offsets front_offset, Offsets back_offset)
        : left(left_port), right(right_port), front(front_port), back(back_port),
          left_offset(left_offset), right_offset(right_offset),
          front_offset(front_offset), back_offset(back_offset) {}

    DistanceSensors(pros::Distance& left_sensor, pros::Distance& right_sensor,
                    pros::Distance& front_sensor, pros::Distance& back_sensor,
                    Offsets left_offset, Offsets right_offset,
                    Offsets front_offset, Offsets back_offset)
        : left(left_sensor), right(right_sensor), front(front_sensor), back(back_sensor),
          left_offset(left_offset), right_offset(right_offset),
          front_offset(front_offset), back_offset(back_offset) {}
    
    pros::Distance& getLeftSensor() { return left; }
    pros::Distance& getRightSensor() { return right; }
    pros::Distance& getFrontSensor() { return front; }
    pros::Distance& getBackSensor() { return back; }

    double getLeftDistance(bool inches = true) {
        return inches ? left.get() * mm_to_in : left.get();
    }
    double getRightDistance(bool inches = true) {
        return inches ? right.get() * mm_to_in : right.get();
    }
    double getFrontDistance(bool inches = true) {
        return inches ? front.get() * mm_to_in : front.get();
    }
    double getBackDistance(bool inches = true) {
        return inches ? back.get() * mm_to_in : back.get();
    }

    Offsets left_offset;
    Offsets right_offset;
    Offsets front_offset;
    Offsets back_offset;

private:
    pros::Distance left;
    pros::Distance right;
    pros::Distance front;
    pros::Distance back;

};