/*
 * bldc.cpp
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#include "string.h"  // for memset
#include "motor.h"
#include "sine_table.h"


void Motor::setup_timer() {
    timer.Instance->PSC = timer_psc;
    timer.Instance->ARR = timer_arr;
    timer.Instance->CCR1 = 0;
    timer.Instance->CCR2 = 0;
    timer.Instance->CCR3 = 0;

    HAL_TIM_Base_Start_IT(&timer);

    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_1);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_2);
    HAL_TIMEx_PWMN_Start(&timer, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&timer, TIM_CHANNEL_3);
}


void Motor::setup_adc() {
	HAL_TIM_OC_Start(&timer, TIM_CHANNEL_6);
	timer.Instance->CCR6 = timer_arr - 1;  // this is where i see the signals best
	HAL_ADC_Start_DMA(&adc, reinterpret_cast<uint32_t*>(&adc_buffer), 4);
}


inline float Motor::calculate_supply_voltage(uint16_t sample) {
    // ADC reference is 3.3V, 12-bit (4095 max)
    constexpr float vref = 3.3f;
    constexpr float adc_max = 4095.0f;

    // voltage divider:
    //  Vout = Vin * (R_down / (R_up + R_down))
    //  => Vin = Vout * ((R_up + R_down) / R_down)
    constexpr float r_up = 51'000.0f;              // R92
    constexpr float r_down = 2'700.0f + 3'300.0f;  // R91 + R93
    constexpr float divider_gain = (r_up + r_down) / r_down;

    float v_adc = (sample / adc_max) * vref;
    return v_adc * divider_gain;
}


inline float Motor::calculate_adc_current(int16_t sample) {
    // ADC reference is 3.3V, 12-bit
    constexpr float vref = 3.3f;
    constexpr float adc_max = 4095.0f;

    // shunt & amplifier configuration:
    // differential amplifier with R_in = 3.3k, Rf = 50k
    // Gain = Rf / Rin = 50k / 3.3k = 15.15
    // V_out = Gain * (V_shunt_pos - V_shunt_neg)
    // I = V_shunt / R_shunt = (V_out / Gain) / R_shunt
    constexpr float r_shunt = 0.002f;   // 2 mΩ
    constexpr float r_in = 3'300.0f;    // R87 or R88
    constexpr float r_feedback = 50'000.0f; // R44
    constexpr float amplifier_gain = r_feedback / r_in;  // 15.15

    float v_out = (sample / adc_max) * vref;
    float v_shunt = v_out / amplifier_gain;
    float current = v_shunt / r_shunt;  // amps

    return current;
}


void Motor::init() {
    setup_timer();
    setup_adc();

    // Optionally prime zero offsets to 0; they will be calibrated on first capture
    current_a_zero_offset = 0;
    current_b_zero_offset = 0;
    current_c_zero_offset = 0;

    HAL_Delay(2);  // wait 1ms guaranteed for some samples to come in


    calibrate_adc();


    // start PWM outputs already started in setup_timer
}


void Motor::calibrate_adc() {
	constexpr auto N = 64;
	uint32_t current_a_average = 0;
	uint32_t current_b_average = 0;
	uint32_t current_c_average = 0;

	for (int i = 0; i < N; i++) {
		current_a_average += adc_buffer.current_a;
		current_b_average += adc_buffer.current_b;
		current_c_average += adc_buffer.current_c;
		HAL_Delay(1);  // wait for next ADC update (blocking)
	}

	current_a_zero_offset = static_cast<int16_t>(current_a_average / N);
	current_b_zero_offset = static_cast<int16_t>(current_b_average / N);
	current_c_zero_offset = static_cast<int16_t>(current_c_average / N);
}


void Motor::adc_calculate() {
    // This is called from the timer update ISR context.
    // Copy last DMA samples (adc_buffer is written by DMA, make a local copy quickly)
    const Adc_dma local = adc_buffer;  // small struct, cheap copy

    // convert ADC samples to physical values using user-supplied functions
    supply_voltage = calculate_supply_voltage(local.supply);

    // subtract zero offsets (interpretation of offsets depends on ADC setup)
    const int16_t a_raw = static_cast<int16_t>(local.current_a) - current_a_zero_offset;
    const int16_t b_raw = static_cast<int16_t>(local.current_b) - current_b_zero_offset;
    const int16_t c_raw = static_cast<int16_t>(local.current_c) - current_c_zero_offset;

    current_a = calculate_adc_current(static_cast<uint16_t>(a_raw));
    current_b = calculate_adc_current(static_cast<uint16_t>(b_raw));
    current_c = calculate_adc_current(static_cast<uint16_t>(c_raw));
}


void Motor::timer_isr() {
	adc_calculate();

	angle += angle_increment;
//	if (angle > 2*TWO_PI) {
//		angle -= TWO_PI;
//	}
//	if (angle < -2*TWO_PI) {
//		angle += TWO_PI;
//	}

	if (supply_voltage > 10.0f) {
		assign_pwm(power, angle);
	} else {
		assign_stop();
	}

}


inline void Motor::assign_pwm(float power, float angle) {
    constexpr float ONE_THIRD_PI = TWO_PI / 3.0f;

    // enforce limits immediately
    if (power < 0.0f) power = 0.0f;
    if (power > 1.0f) power = 1.0f;

    // precompute duty scaling
    constexpr uint32_t min_ticks = 150;
    constexpr uint32_t max_ticks = timer_arr - min_ticks;
    constexpr float half_period = static_cast<float>(timer_arr) / 2.0f;

    // get normalized sine phase values
    const float sa = fast_sin(angle);
    const float sb = fast_sin(angle - ONE_THIRD_PI);
    const float sc = fast_sin(angle + ONE_THIRD_PI);

    // convert to duty ticks (center-aligned, bipolar to unipolar mapping)
    auto scale_to_ticks = [&](float x) -> uint32_t {
        // normalized sine -1..1 mapped to 0..timer_arr
        float duty = (x * power * half_period) + half_period;
        // enforce clamp limits
        if (duty < static_cast<float>(min_ticks))
            duty = static_cast<float>(min_ticks);
        else if (duty > static_cast<float>(max_ticks))
            duty = static_cast<float>(max_ticks);
        return static_cast<uint32_t>(duty);
    };

    // compute phase PWM compare values
    const uint32_t ccr_a = scale_to_ticks(sa);
    const uint32_t ccr_b = scale_to_ticks(sb);
    const uint32_t ccr_c = scale_to_ticks(sc);

    // write CCR registers directly (low-latency)
    timer.Instance->CCR1 = ccr_a;
    timer.Instance->CCR2 = ccr_b;
    timer.Instance->CCR3 = ccr_c;
}


inline void Motor::assign_stop() {
    timer.Instance->CCR1 = 0;
    timer.Instance->CCR2 = 0;
    timer.Instance->CCR3 = 0;
}

