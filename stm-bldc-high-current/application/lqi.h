/*
 * lqi.h
 *
 *  Created on: Nov 10, 2025
 *      Author: ftobler
 */

#pragma once


/**
 * Implements a Linear Quadratic Integrator (LQI) controller
 * for single-axis position tracking.
 */
class LqiController {
private:
    // LQR Gains
    const float K_p;        // K1: Position Error Gain
    const float K_w;        // K2: Speed Error Gain (Damping)
    const float K_i;        // K3: Integral Error Gain

    // Target values (fed from the Shaper)
    float target_position = 0.0f;
    float target_speed = 0.0f;

    // System Parameters
    const float DT;           // Control loop time step (seconds)
    const float MAX_CURRENT;  // Max commanded current for saturation

    // LQI State History
    float integral_error;  // The accumulated integral of position error (x_int)

    // Safety Limit for Integral Accumulation
    float max_integral = 1000.0f;

    inline float clamp(float value, float min_val, float max_val) {
        if (value < min_val) return min_val;
        if (value > max_val) return max_val;
        return value;
    }

public:

    LqiController(float Kp, float Kw, float Ki, float dt, float max_current)
        : K_p(Kp), K_w(Kw), K_i(Ki), DT(dt), MAX_CURRENT(max_current) {
        integral_error = 0.0f;
    }

    void reset();


    float update(float act_position, float act_speed);

    /**
     * Sets the position setpoint (theta_ref).
     */
    void set_position(float position) { target_position = position; };  // FIX: Used 'position'

    /**
     * Sets the speed setpoint (omega_ref).
     */
    void set_speed(float speed) { target_speed = speed; };

    /**
     * Sets the saturation limit for the integral term.
     */
    void set_max_integral(float limit) { max_integral = limit; };

};
