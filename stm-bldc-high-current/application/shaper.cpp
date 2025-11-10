/*
 * shaper.cpp
 *
 *  Created on: Nov 9, 2025
 *      Author: ftobler
 */

#include "shaper.h"
#include "math.h"


using namespace std;


void Shaper::update() {
    // Remaining distance
    float dist = target - current_position;
    float dist_abs = absf(dist);

    // If already at target (within one time-step reachable tolerance), snap to it.
    // Tolerance can be DT*max_velocity to ensure we don't choke on floating rounding.
    const float tol = max_velocity * DT * 0.5f;
    if (dist_abs <= tol) {
        current_position = target;
        current_speed = 0.0f;
        return;
    }

    // Maximum speed that still allows stopping within remaining distance:
    // v_stop = sqrt(2 * a * d)
    // if a == 0, treat as immediate limit to prevent div-by-zero
    float v_stop = 0.0f;
    if (max_accel > 0.0f) {
        v_stop = sqrtf(2.0f * max_accel * dist_abs);
    } else {
        v_stop = 0.0f;  // if no accel capability, best to keep speed 0
    }

    // Allowed speed magnitude considering both top speed and braking distance
    float allowed_speed_mag = (max_velocity < v_stop) ? max_velocity : v_stop;

    // Desired speed with sign toward target
    float desired_speed = allowed_speed_mag * sgn(dist);

    // Limit how much speed can change this step (acceleration constraint)
    float max_delta_v = max_accel * DT;
    float delta_v = desired_speed - current_speed;
    delta_v = clampf(delta_v, -max_delta_v, max_delta_v);

    current_speed += delta_v;

    // If very small speed due to rounding, zero it
    if (absf(current_speed) < 1e-6f) current_speed = 0.0f;

    // Predict next position
    float next_position = current_position + current_speed * DT;

    // If the update would cross the target, snap to target and zero speed
    if ((target - current_position) * (target - next_position) <= 0.0f) {
        current_position = target;
        current_speed = 0.0f;
    } else {
        current_position = next_position;
    }
}
