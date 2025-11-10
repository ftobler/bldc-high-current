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

}
