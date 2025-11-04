/*
 * position_control.cpp
 *
 *  Created on: Nov 4, 2025
 *      Author: ftobler
 */

#include "position_control.h"

Position::Position() {

}


void Position::start(int32_t current_position) {
    speed_integrator = 0.0f;
    speed_prev_error = 0.0f;
    target_speed = 0;
    prev_position = current_position;
}


void Position::set_target(int32_t target) {
    target_speed = target;
}


float Position::update(int32_t current_position) {
    // Position loop
//    float pos_error = static_cast<float>(target_position) - static_cast<float>(current_position);
//    float target_speed = pos_error * pos_p;

    // Calculate current speed
    float current_speed = static_cast<float>(current_position - prev_position) * 24000.0f / 4096.0f;
    prev_position = current_position;
    current_speed_filtered = 0.9f * current_speed_filtered + 0.1f * current_speed;

    // Speed loop
    float speed_error = target_speed - current_speed_filtered;
    speed_integrator += speed_error;
//    if (speed_integrator > max_integrator) {
//        speed_integrator = max_integrator;
//    } else if (speed_integrator < -max_integrator) {
//        speed_integrator = -max_integrator;
//    }

    float speed_derivative = speed_error - speed_prev_error;
    speed_prev_error = speed_error;

    float output = speed_error * speed_p + speed_integrator * speed_i + speed_derivative * speed_d;

    return output;
}
