/*
 * calibration.h
 *
 *  Created on: Nov 2, 2025
 *      Author: ftobler
 */

#pragma once


#include "stdint.h"
#include "sine_table.h"


class Calibration {
private:
	enum class State {
		idle,
		find_zero,
		go_to_start,
		sweep,
		done
	};

	// simple constants for this calibration routine
	static constexpr float CAL_PWM = 0.15f;          // 15% duty during calibration
	static constexpr int   CAL_ELEC_STEPS = 4096*8;  // number of electrical steps
	static constexpr int   POLE_PAIRS = 5;           // motor-specific!
	static constexpr float CAL_REV = 2.0f * PI;      // one electrical revolution
	static constexpr float CAL_STEP_ANGLE = CAL_REV / CAL_ELEC_STEPS;

	State state = State::idle;
	int step = 0;

	float output_power = 0.0f;
	float angle_value = 0.0f;

	// encoder accumulation
	int64_t accum_encoder = 0;
	int accum_count = 0;
	float offset = 0.0f;


public:
	void start();
	void update(int32_t encoder_value);

	inline float power() {
		return output_power;
	}

	inline float angle() {
		return angle_value;
	}

	inline bool done() {
		return state == State::done;
	}
};
