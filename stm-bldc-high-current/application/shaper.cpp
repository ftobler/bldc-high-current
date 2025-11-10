/*
 * shaper.cpp
 *
 *  Created on: Nov 9, 2025
 *      Author: ftobler
 */

#include "shaper.h"


using namespace std;


void Shaper::update() {
    float old_current_position = current_position;
    float delta_to_target = target - current_position;

    if (abs(delta_to_target) > max_delta) {
        // If the error is large, move by exactly max_delta
        current_position += (delta_to_target > 0) ? max_delta : -max_delta;
    } else {
        // If we are close, snap exactly to the final target
        current_position = target;
    }

    // Calculate the instantaneous required speed (omega_ref)
    // Speed = (New Position - Old Position) / Time Step
    current_speed = (current_position - old_current_position) / DT;
}

