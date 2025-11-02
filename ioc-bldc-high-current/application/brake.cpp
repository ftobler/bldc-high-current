/*
 * brake.cpp
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#include "brake.h"


void Brake::init() {
	HAL_TIM_Base_Start(&timer);
	HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
}


void Brake::update(float voltage) {
	constexpr float start_voltage = 25.0f;
	constexpr float end_voltage = 26.0f;
	constexpr float span_voltage = end_voltage - start_voltage;
	constexpr float gain = 1 / span_voltage;
	const float amount =  (voltage - start_voltage) * gain;  // normalized from 0..1
	constexpr uint32_t timer_max = 0x4000 - 1;
	if (amount > 1.0f) {
		timer.Instance->CCR1 = timer_max;
	} else if (amount <= 0.0f) {
		timer.Instance->CCR1 = 0;
	} else {
		timer.Instance->CCR1 = static_cast<uint32_t>(amount * timer_max);
	}
}

