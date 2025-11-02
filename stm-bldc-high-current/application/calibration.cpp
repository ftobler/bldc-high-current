/*
 * calibration.cpp
 *
 *  Created on: Nov 2, 2025
 *      Author: ftobler
 */

#include "calibration.h"


void Calibration::start() {
	state = State::find_zero;
	step = 0;
	output_power = 0.0f;
	angle_value = 0.0f;
	accum_encoder = 0;
	accum_count = 0;
	offset = 0.0f;
}


// called each control loop / ISR with encoder reading
void Calibration::update(int32_t encoder_value) {
	switch (state) {
	case State::idle:
	case State::done:
		output_power = 0.0f;
		break;

	case State::find_zero:
		if (encoder_value >= 0 && encoder_value < 10) {
			state = State::go_to_start;
		} else {
			if (encoder_value > 0) {
				angle_value -= CAL_STEP_ANGLE;
				if (angle_value < TWO_PI) angle_value += TWO_PI;
			} else {
				angle_value += CAL_STEP_ANGLE;
				if (angle_value > TWO_PI) angle_value -= TWO_PI;
			}
		}
		output_power = CAL_PWM / 2.0f;
		break;

	case State::go_to_start:
		angle_value += CAL_STEP_ANGLE;
		if (angle_value > TWO_PI) {
			angle_value = 0;
			state = State::sweep;
		}
		output_power = CAL_PWM / 2.0f;
		break;

	case State::sweep:
		angle_value = step * CAL_STEP_ANGLE;
		output_power = CAL_PWM;
		accum_encoder += encoder_value;
		accum_count++;

		step++;
		if (step >= CAL_ELEC_STEPS * ELECTRICAL_SWEEP_REVOLUTIONS) {
			float avg = static_cast<float>(accum_encoder) / static_cast<float>(accum_count);
			offset = avg;  // this will be PI rotor angle
			output_power = 0.0f;
			state = State::done;
		}
		break;
	}
}
