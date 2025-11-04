/*
 * position_control.h
 *
 *  Created on: Nov 4, 2025
 *      Author: ftobler
 */

#pragma once


#include "stdint.h"


class Position {
private:
    // Position loop gains
//    float pos_p = 0.1f;

    // Speed loop gains
    float speed_p = 0.5f;
    float speed_i = 0.0002f;
    float speed_d = 0.000f;
//    constexpr auto max_integrator = 45.00f;

    float target_speed = 0;
    int32_t prev_position = 0;

    float speed_integrator = 0.0f;
    float speed_prev_error = 0.0f;

    float current_speed_filtered = 0.0f;

public:
    Position();
    void start(int32_t current_position);  // call to re-init control loop
    void set_target(int32_t target);

    /**
     * input current position reading (encoder value)
     * output motor drive current
     */
    float update(int32_t current_position);
};

