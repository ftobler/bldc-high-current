/*
 * position_control.cpp
 *
 *  Created on: Nov 4, 2025
 *      Author: ftobler
 */

#include "speedometer.h"



void Speedometer::start(int32_t current_position) {
    prev_position = current_position;
}


float Speedometer::update(int32_t current_position) {
    const float current_speed = static_cast<float>(current_position - prev_position) * sample_rate / 4096.0f;
    prev_position = current_position;
    const float filtered = 0.995f * current_speed_filtered + 0.005f * current_speed;

    current_speed_filtered = filtered;
    return filtered;

//    // Speed loop
//    float speed_error = target_speed - current_speed;
//    float speed_error_filtered = target_speed - current_speed_filtered;
//    speed_integrator += speed_error;
////    if (speed_integrator > max_integrator) {
////        speed_integrator = max_integrator;
////    } else if (speed_integrator < -max_integrator) {
////        speed_integrator = -max_integrator;
////    }
//
//    float speed_derivative = speed_error - old_error;
//    old_error = speed_error;
//
//    float output = speed_error_filtered * speed_p + speed_integrator * speed_i + speed_derivative * speed_d;
//
//    constexpr auto max_output = 35.0f;
//    if (output > max_output) {
//        return max_output;
//    }
//    if (output < -max_output) {
//        return -max_output;
//    }
//    return output;
}
