/*
 * position_control.cpp
 *
 *  Created on: Nov 4, 2025
 *      Author: ftobler
 */

#include "position_control.h"



void Position::start(int32_t current_position) {
    position_integrator = 0.0f;
    old_error = 0;
}


float Position::update(int32_t current_position) {
    float error = target_position - current_position;
    position_integrator += error;

    constexpr auto integrator_max = 1200000.0f;
    if (position_integrator >  integrator_max) position_integrator =  integrator_max;
    if (position_integrator < -integrator_max) position_integrator = -integrator_max;

    float position_derivative = error - old_error;
    old_error = error;

    float output = error * position_p + position_integrator * position_i + position_derivative * position_d;

    constexpr auto max_output = 12.0f;
    if (output >  max_output) return  max_output;
    if (output < -max_output) return -max_output;
    return output;
}
