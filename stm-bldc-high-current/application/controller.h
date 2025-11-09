/*
 * controller.h
 *
 *  Created on: Nov 9, 2025
 *      Author: ftobler
 */

#pragma once


class Controller {
private:
    // static values (non static to trim in debugger), otherwise set in constructor
    float kp;
    float ki;
    float integrator_decay;  // exponential decay rate [1/s], 0 = none
    float out_max;
    float out_min;
    float sample_time;

    // internal state variables
    float integrator = 0.0f;
    float integrator_max = 0.0f;  // calculated from out_max

    //
    float target = 0.0f;  // target value
    float cost_accum = 0.0f;  // cost calculation (square of error)
    float last_output = 0.0f;

    // helper
    static inline float clamp(float v, float a, float b) {
        return (v < a) ? a : ((v > b) ? b : v);
    }
    void recompute_integrator_limits();
public:
    Controller(float kp = 0.0f,
               float ki = 0.0f,
               float integrator_decay = 0.0f,
               float out_max = 1.0f,
               float out_min = 0.0f,
               float sample_time = 0.001f);
    void start();
    float update(float measurement);
    void set_target(float new_target) { target = new_target; }
};


