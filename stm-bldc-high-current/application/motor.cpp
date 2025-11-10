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


inline void Motor::setup_adc() {
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


inline void Motor::calibrate_adc() {
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


inline void Motor::adc_calculate() {
    // This is called from the timer update ISR context.
    // Copy last DMA samples (adc_buffer is written by DMA, make a local copy quickly)
    const Adc_dma local = adc_buffer;  // small struct, cheap copy

    // convert ADC samples to physical values using user-supplied functions
    supply_voltage = calculate_supply_voltage(local.supply);

    // subtract zero offsets (interpretation of offsets depends on ADC setup)
    const int16_t a_raw = static_cast<int16_t>(local.current_a) - current_a_zero_offset;
    const int16_t b_raw = static_cast<int16_t>(local.current_b) - current_b_zero_offset;
    const int16_t c_raw = static_cast<int16_t>(local.current_c) - current_c_zero_offset;

    float new_current_a = calculate_adc_current(static_cast<uint16_t>(a_raw));
    float new_current_b = calculate_adc_current(static_cast<uint16_t>(b_raw));
    float new_current_c = calculate_adc_current(static_cast<uint16_t>(c_raw));

    current_kirchhoff_reconstruct(new_current_a, new_current_b, new_current_c);

    // tp filtered version
    constexpr float filter_new = 0.33f;
    constexpr float filter_old = 1.0f - filter_new;
    current_a_tpfilter = current_a_tpfilter * filter_old + current_a * filter_new;
    current_b_tpfilter = current_b_tpfilter * filter_old + current_b * filter_new;
    current_c_tpfilter = current_c_tpfilter * filter_old + current_c * filter_new;
}


inline void Motor::current_kirchhoff_reconstruct(float a, float b, float c) {
//    if (a > c && b > c) {
//        // c is the smallest
//        current_a = a;
//        current_b = b;
//        current_c = -a - b;
//    } else if (a > b && c > b) {
//        // b is the smallest
//        current_a = a;
//        current_b = -a - c;
//        current_c = c;
//    } else {
//        // a is the smallest
//        current_a = -b - c;
//        current_b = b;
//        current_c = c;
//    }

//    if (a < c && b < c) {
//        // c is the biggest
//        current_a = a;
//        current_b = b;
//        current_c = -a - b;
//    } else if (a < b && c < b) {
//        // b is the biggest
//        current_a = a;
//        current_b = -a - c;
//        current_c = c;
//    } else {
//        // a is the biggest
//        current_a = -b - c;
//        current_b = b;
//        current_c = c;
//    }

//    if (a > c && c > b) {
//        // c is mid
//        current_a = a;
//        current_b = b;
//        current_c = -a - b;
//    } else if (a > b && b > c) {
//        // b is mid
//        current_a = a;
//        current_b = -a - c;
//        current_c = c;
//    } else {
//        // a is mid
//        current_a = -b - c;
//        current_b = b;
//        current_c = c;
//    }

//    current_a = a;
//    current_b = -a - c;
//    current_c = c;

//   const float mean = (a + b + c) / 3.0f / 3.0f;
//   current_a = a - mean;
//   current_b = b - mean;
//   current_c = c - mean;

    current_a = a;
    current_b = b;
//    current_b = -a - c;  // b is defect
    current_c = c;
}


void Motor::timer_isr() {
    adc_calculate();

//    if (angle > 2*TWO_PI) {
//        angle -= TWO_PI;
//    }
//    if (angle < -2*TWO_PI) {
//        angle += TWO_PI;
//    }

    if (pwm_on) {
        if (supply_voltage < 10.0f) pwm_on = false;
    } else {
        if (supply_voltage > 11.0f) pwm_on = true;
    }

    if (!pwm_on) {
        assign_stop();
        return;
    }

    switch (state) {
        case Mode::off:
            angle = 0;
            power = 0;
            assign_stop();
            break;

        case Mode::manual:
            angle += angle_increment;
            assign_pwm(power, angle);
            break;

        case Mode::calibration:
            run_calibration_step();
            break;

        case Mode::power_control:
            run_power_control();
            break;

        case Mode::current_control:
            run_current_control();
            break;

        case Mode::speed_control:
            run_speed_control();
            break;

        case Mode::position_control:
            run_position_control();
            break;

        case Mode::shaped_position:
            run_position_shaped_control();
            break;

        case Mode::lqi_position:
            run_lqi_control();
            break;

        default:
            state = Mode::off;
            stop_reason = StopReason::none;
    }
    old_state = state;

}


inline void Motor::assign_pwm(float power, float angle) {
    constexpr float ONE_THIRD_PI = TWO_PI / 3.0f;

    // enforce limits immediately
    if (power < 0.0f) power = 0.0f;
    if (power > 1.0f) power = 1.0f;

    // get normalized sine phase values
    const float sa = fast_sin(angle);
    const float sb = fast_sin(angle - ONE_THIRD_PI);
    const float sc = fast_sin(angle + ONE_THIRD_PI);

    assign_pwm_volt(sa * power, sb * power, sc * power);
}


inline void Motor::assign_pwm_volt(float sa, float sb, float sc) {
    // precompute duty scaling
    constexpr uint32_t min_ticks = 150;
    constexpr uint32_t max_ticks = timer_arr - min_ticks;
    constexpr float half_period = static_cast<float>(timer_arr) / 2.0f;

    // convert to duty ticks (center-aligned, bipolar to unipolar mapping)
    auto scale_to_ticks = [&](float x) -> uint32_t {
        // normalized sine -1..1 mapped to 0..timer_arr
        float duty = (x * half_period) + half_period;
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


inline void Motor::run_calibration_step() {
    if (old_state != state) {
        // init calibration
        calibration.start();
    } else {
        calibration.update(encoder_value);
        assign_pwm(calibration.power(), calibration.angle());
        if (calibration.done()) {
            stop_reason = StopReason::success;
            state = Mode::off;
        }
    }
}


inline void Motor::run_power_control() {
    constexpr float gain = pole_pairs * TWO_PI / 4096.0;
    float current_angle = (encoder_value - encoder_offset) * gain + PI;
    if (power > 0) {
        angle = current_angle + PI / 2;
        assign_pwm(power, angle);
    } else {
        angle = current_angle - PI / 2;
        assign_pwm(-power, angle);
    }
}


inline void Motor::run_current_control() {
    // detect state change and if needed re-start the control loops
    if (old_state != state) {
        foc.start();
    }

    // calculate the current angle from the encoder. we have a calibrated encoder value
    // said value needs to be shifted some round fraction of pi
    // if id and iq targets appear to be swapped change this offset plus or minus 0.5pi to 1pi
    constexpr float gain = pole_pairs * TWO_PI / 4096.0;
    float current_angle = (encoder_value - encoder_offset) * gain - PI / 2.0f;

    // using the foc algorythm (park clarke) re-evaluate pwm voltags
    Vector3 v = foc.update(-current_a, -current_b, -current_c, current_angle);

    // assign pwm
    pwm_safe_assign(v);

}


inline void Motor::run_speed_control() {
    if (old_state != state) {
        speedometer.start(encoder_value);
        speed.start();
        foc.set_id(0.0f);
    }

    const float speed_value = speedometer.update(encoder_value);
    const float output = speed.update(speed_value);
    foc.set_iq(-output);

    run_current_control();
}


inline void Motor::run_position_control() {
    if (old_state != state) {
        speedometer.start(encoder_value);
        speed.start();
        position.start();
        position.set_target(encoder_value / 4096.0f);
        foc.set_id(0.0f);
    }

    const float out = position.update(encoder_value / 4096.0f);
    speed.set_target(out);

    run_speed_control();
}


inline void Motor::run_position_shaped_control() {
    if (old_state != state) {
        shaper.start(encoder_value / 4096.0f);
    }
    shaper.update();
    position.set_target(shaper.get_position());
    run_position_control();
}


inline void Motor::run_lqi_control() {
    if (old_state != state) {
        shaper.start(encoder_value / 4096.0f);
        speedometer.start(encoder_value);
        foc.set_id(0.0f);
        lqi.reset();
    }
    shaper.update();
    const float speed_value = speedometer.update(encoder_value);
    lqi.set_position(shaper.get_position());
    lqi.set_speed(shaper.get_speed());
    const float output = lqi.update(encoder_value / 4096.0f, speed_value);
    foc.set_iq(output);

    run_current_control();
}


/**
 * apply pwm voltages with safe emergency stop conditions
 */
inline void Motor::pwm_safe_assign(Vector3& v) {
    // lambda helper for safety limits
    constexpr float max_current = 50.0f;
//    constexpr float max_pwm = 0.95f;

    // lambda helper for single value out of bounds
    auto outbound = [&](float x, float max) -> bool {
        return x > max || x < -max;
    };
    // helper for triple value out of bounds
    auto outbound3 = [&](float a, float b, float c, float max) -> bool {
        return outbound(a, max) || outbound(b, max) || outbound(c, max);
    };

//    const bool pwm_out_of_bound = outbound3(v.a, v.b, v.c, max_pwm);
    constexpr bool pwm_out_of_bound = false;
    const bool current_out_of_bound = outbound3(current_a_tpfilter, current_b_tpfilter, current_c_tpfilter, max_current);

    if (pwm_out_of_bound || current_out_of_bound) {
        assign_stop();  // emergency stop
        if (pwm_out_of_bound) {
            stop_reason = StopReason::pwm;
        }
        if (current_out_of_bound) {
            stop_reason = StopReason::current;
        }
        state = Mode::off;
    } else {
        assign_pwm_volt(v.a, v.b, v.c);
    }
}

