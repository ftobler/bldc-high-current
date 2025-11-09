/*
 * controller.cpp
 *
 *  Created on: Nov 9, 2025
 *      Author: ftobler
 */


#include "controller.h"
#include "math.h"


using namespace std;


Controller::Controller(float kp,
                       float ki,
                       float integrator_decay,
                       float out_max,
                       float out_min,
                       float sample_time)
    : kp(kp),
      ki(ki),
      integrator_decay(integrator_decay),
      out_max(out_max),
      out_min(out_min),
      sample_time(sample_time) {
    recompute_integrator_limits();
}


void Controller::start() {
    target = 0;
    integrator = 0.0f;
    last_output = 0.0f;
    cost_accum = 0.0f;
    recompute_integrator_limits();
}


void Controller::recompute_integrator_limits() {
    // Compute limits so that integral term alone cannot drive output past its limits
    float max_max = fabs(out_max / ki);
    float max_min = fabs(out_min / ki);
    integrator_max = fmax(max_max, max_min);
}


float Controller::update(float measurement) {
    // build error
    const float error = target - measurement;

    // update error cost accumulator (square error)
    cost_accum = cost_accum * 0.99995f + error * error * sample_time;

    // integrate the error (with windup)
    const float integrator_new = integrator + error * sample_time;
    const float clamped_integrator =  clamp(integrator_new * integrator_decay, -integrator_max, integrator_max);
    integrator = clamped_integrator;

    // calculate and clamp output
    const float output = clamp(clamped_integrator * ki + error * kp, out_min, out_max);

    // assign/return
    last_output = output;
    return output;
}
