/*
 * position_control.h
 *
 *  Created on: Nov 4, 2025
 *      Author: ftobler
 */

#pragma once


#include "stdint.h"


class Speedometer {
private:
//    float speed_p = 1.8f;
//    float speed_i = 0.001f;
//    float speed_d = 0.000f;

//    float target_speed = 0;
    int32_t prev_position = 0;

//    float speed_integrator = 0.0f;
//    float old_error = 0.0f;

    float current_speed_filtered = 0.0f;
    float sample_rate;


public:
    Speedometer(float sample_time): sample_rate(1 / sample_time) {}

    void start(int32_t current_position);  // call to re-init control loop
//    void set_target(float target) {
//        target_speed = target;
//    }

    /**
     * input current position reading (encoder value)
     * output motor drive current
     */
    float update(int32_t current_position);
};

