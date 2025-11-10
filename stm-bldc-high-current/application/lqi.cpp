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


float LqiController::update(float act_position, float act_speed) {
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
