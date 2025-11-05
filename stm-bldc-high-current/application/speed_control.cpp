/*
 * position_control.cpp
 *
 *  Created on: Nov 4, 2025
 *      Author: ftobler
 */

#include "speed_control.h"



void Speed::start(int32_t current_position) {
    speed_integrator = 0.0f;
    old_error = 0.0f;
    target_speed = 0;
    prev_position = current_position;
}


float Speed::update(int32_t current_position) {
    float current_speed = static_cast<float>(current_position - prev_position) * 24000.0f / 4096.0f;
    prev_position = current_position;
    current_speed_filtered = 0.995f * current_speed_filtered + 0.005f * current_speed;

    // Speed loop
    float speed_error = target_speed - current_speed;
    float speed_error_filtered = target_speed - current_speed_filtered;
    speed_integrator += speed_error;
//    if (speed_integrator > max_integrator) {
//        speed_integrator = max_integrator;
//    } else if (speed_integrator < -max_integrator) {
//        speed_integrator = -max_integrator;
//    }

    float speed_derivative = speed_error - old_error;
    old_error = speed_error;

    float output = speed_error_filtered * speed_p + speed_integrator * speed_i + speed_derivative * speed_d;

    constexpr auto max_output = 35.0f;
    if (output > max_output) {
        return max_output;
    }
    if (output < -max_output) {
        return -max_output;
    }
    return output;
}
