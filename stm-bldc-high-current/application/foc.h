/*
 * foc.h
 *
 *  Created on: Nov 2, 2025
 *      Author: ftobler
 */

#pragma once

#include "controller.h"


struct Vector3 {
    float a;
    float b;
    float c;
};


class Foc {
private:

    // --- control gains (tune later) ---
//    float kp_d = 0.005f;
//    float ki_d = 0.001f;
//    float kp_q = 0.005f;
//    float ki_q = 0.001f;

    // --- PI integrators ---
//    float integrator_d = 0.0f;
//    float integrator_q = 0.0f;

    // --- targets ---
//    float target_id = 0.0f;
//    float target_iq = 0.0f;

    // --- results (for debugging / visibility) ---
    float id = 0.0f;
    float iq = 0.0f;
    float debug_angle = 0.0f;

    Controller controller_d;
    Controller controller_q;

    inline void clarke_transform(float a, float b, float c, float& alpha, float& beta);
    inline void inv_clarke_transform(float alpha, float beta, float& a, float& b, float& c);
    inline void park_transform(float alpha, float beta, float angle, float& d, float& q);
    inline void inv_park_transform(float d, float q, float angle, float& alpha, float& beta);
public:
    Foc(): controller_d(0.04f/2, 24.0f, 0.9999f, 40.0f, -40.0f, 1/24000.0f),
           controller_q(0.04f/2, 24.0f, 0.9999f, 40.0f, -40.0f, 1/24000.0f) {};
    void start();
    Vector3 update(float current_a, float current_b, float current_c, float angle);
    void set_id(float new_id) { controller_d.set_target(new_id); }
    void set_iq(float new_iq) { controller_q.set_target(new_iq); }
};
