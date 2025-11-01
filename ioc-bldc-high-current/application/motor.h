/*
 * bldc.h
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#pragma once

#include "stm32_hal.h"


 struct  __attribute__((packed)) Adc_dma {
	uint16_t current_a;
	uint16_t current_b;
	uint16_t current_c;
	uint16_t supply;
};

static_assert(sizeof(Adc_dma) == 8);  // if false, compiler is set up wrong


class Motor {
private:
	static constexpr uint32_t timer_psc = 0;
	static constexpr uint32_t mcu_frequency = 170000000;
	static constexpr uint32_t pwm_frequency = 24000;
	static constexpr uint32_t timer_arr = mcu_frequency / pwm_frequency / 2;  // div 2 is because we are using center aligned mode

	TIM_HandleTypeDef& timer;
	ADC_HandleTypeDef& adc;
	Adc_dma adc_buffer = {0};

	// at startup, the first sample is to calibrate 0A point
	int16_t current_a_zero_offset = 0;
	int16_t current_b_zero_offset = 0;
	int16_t current_c_zero_offset = 0;

	// runtime control variables
	float power = 0.0f;
	float angle = 0.0f;
	float angle_increment = 0.0f;

	// calculated phase currents
	float current_a = 0.0f;
	float current_b = 0.0f;
	float current_c = 0.0f;
	float supply_voltage = 0.0f;
	inline void assign_pwm(float power, float angle);
	inline void assign_stop();
	inline void setup_timer();
	inline void setup_adc();
	inline float calculate_supply_voltage(uint16_t sample);
	inline float calculate_adc_current(int16_t sample);
	inline void adc_calculate();
	void calibrate_adc();
public:
	Motor(
			TIM_HandleTypeDef& timer,
			ADC_HandleTypeDef& adc
	):
		timer(timer),
		adc(adc) {};

	/**
	 * called once at startup
	 * needs to start ADC in continious DMA mode
	 * needs to start pwm timer
	 */
	void init();

	/**
	 * timer update isr calls this
	 * it's time to re-calculate the currents from last ADC capture
	 */
	void timer_isr();
};
