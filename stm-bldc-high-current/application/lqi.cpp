/*
 * lqi.cpp
 *
 *  Created on: Nov 10, 2025
 *      Author: ftobler
 */

 #include "lqi.h"


void LqiController::reset() {
    integral_error = 0.0f;
    target_position = 0.0f;
    target_speed = 0.0f;
}


float LqiController::update_PoE(float act_position, float act_speed) {
    // 1. Calculate the State Errors (e_theta and e_omega)
    float error_position = target_position - act_position;
    float error_speed = target_speed - act_speed;

    // 2. Control Law Calculation (LQR State Feedback)
    // u = - (K_p*e_pos + K_w*e_vel + K_i*e_int)
    float lqr_sum = (K_p * error_position) +
                    (K_w * error_speed)    +
                    (K_i * integral_error);

    float iq_ref = -lqr_sum;

    // 3. Apply Saturation (Clamping the output)
    float saturated_iq_ref = clamp(iq_ref, -MAX_CURRENT, MAX_CURRENT);

    // 4. Integral Accumulation with Anti-Windup (Conditional Update)
    if (iq_ref == saturated_iq_ref) {
        // Accumulate if the output is not saturated (i.e., control effort is not maxed out)
        integral_error += error_position * DT;
    }

    // Safety: Clamp the integral term itself
    integral_error = clamp(integral_error, -max_integral, max_integral);

    return saturated_iq_ref;
}


float LqiController::update_PoM(float act_position, float act_speed) {
    // 1. Calculate the State Errors (still used for integral)
    float error_position = target_position - act_position;
    float error_speed = target_speed - act_speed;

    // 2. Control Law Calculation (PoM for position, PoE for speed & integral)
    // Proportional on Measurement: -K_p * act_position
    float pos_term = -K_p * act_position;

    // Speed and integral terms stay proportional on error
    float speed_term = -K_w * act_speed;
    float int_term   = K_i * integral_error;

    float lqr_sum = pos_term + speed_term + int_term;
    float iq_ref = -lqr_sum;

    // 3. Apply Saturation
    float saturated_iq_ref = clamp(iq_ref, -MAX_CURRENT, MAX_CURRENT);

    // 4. Integral Accumulation with Anti-Windup
    if (iq_ref == saturated_iq_ref) {
        integral_error += error_position * DT;
    }

    // Clamp the integral term
    integral_error = clamp(integral_error, -max_integral, max_integral);

    return saturated_iq_ref;
}
