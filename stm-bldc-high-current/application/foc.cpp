/*
 * foc.cpp
 *
 *  Created on: Nov 2, 2025
 *      Author: ftobler
 */

#include "foc.h"
#include "sine_table.h"


/**
 * 3 phases to 2 vectors
 */
inline void Foc::clarke_transform(float a, float b, float c, float& alpha, float& beta) {
    alpha = a;
    constexpr float one_over_sqrt_three = 0.5773502691896258f;  // 1/sqrt(3) (from python)
    beta = one_over_sqrt_three * (b - c);
}


/**
 * 2 vectors to 3 phases
 */
inline void Foc::inv_clarke_transform(float alpha, float beta, float& a, float& b, float& c) {
    a = alpha;
    constexpr float sine_pi_third = 0.8660254037844386f;  // sin(pi/3)  (from python)
    b = -0.5f * alpha + sine_pi_third * beta;
    c = -0.5f * alpha - sine_pi_third * beta;
}


/**
 * rotate 2 vector system
 */
inline void Foc::park_transform(float alpha, float beta, float angle, float& d, float& q) {
    const float s = fast_sin(angle);
    const float c = fast_cos(angle);
    d =  alpha * c + beta * s;
    q = -alpha * s + beta * c;
}


/**
 * rotate 2 vector system (backwards)
 */
inline void Foc::inv_park_transform(float d, float q, float angle, float& alpha, float& beta) {
    const float s = fast_sin(angle);
    const float c = fast_cos(angle);
    alpha = d * c - q * s;
    beta  = d * s + q * c;
}


void Foc::start() {
    integrator_d = 0.0f;
    integrator_q = 0.0f;
}


Vector3 Foc::update(float current_a, float current_b, float current_c, float angle) {
    debug_angle = angle;
    // Step 1: Clarke transform
    float i_alpha, i_beta;
    clarke_transform(current_a, current_b, current_c, i_alpha, i_beta);

    // Step 2: Park transform
    park_transform(i_alpha, i_beta, angle, id, iq);

    // Step 3: PI current control
    const float err_d = target_id - id;
    const float err_q = target_iq - iq;

    integrator_d += err_d * ki_d;
    integrator_q += err_q * ki_q;

    // limit integrators to avoid windup
    constexpr float integrator_limit = 1.0f;
    if (integrator_d > integrator_limit) integrator_d = integrator_limit;
    if (integrator_d < -integrator_limit) integrator_d = -integrator_limit;
    if (integrator_q > integrator_limit) integrator_q = integrator_limit;
    if (integrator_q < -integrator_limit) integrator_q = -integrator_limit;

    const float v_d = kp_d * err_d + integrator_d;
    const float v_q = kp_q * err_q + integrator_q;

    // Step 4: inverse Park transform
    float v_alpha, v_beta;
    inv_park_transform(v_d, v_q, angle, v_alpha, v_beta);

    // Step 5: inverse Clarke -> phase voltages
    Vector3 v;
    inv_clarke_transform(v_alpha, v_beta, v.a, v.b, v.c);

    return v;
}
