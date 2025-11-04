/*
 * bldc.h
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#pragma once

#include "stm32_hal.h"
#include "calibration.h"
#include "foc.h"


 struct  __attribute__((packed)) Adc_dma {
    uint16_t current_a;
    uint16_t current_b;
    uint16_t current_c;
    uint16_t supply;
};

static_assert(sizeof(Adc_dma) == 8);  // if false, compiler is set up wrong


class Motor {
private:
    enum class Mode : uint8_t {
        off,            // safe off
        manual,         // direct set of angle/power from user or debug
        calibration,    // static PWM stepping for encoder alignment
        power_control,  // sets angle automatic +90deg or -90deg
        current_control, // inner FOC or current loop active
        position_control
    };
    enum class StopReason {
        none,
        pwm,
        current,
        success,
    };

    static constexpr uint32_t timer_psc = 0;
    static constexpr uint32_t mcu_frequency = 170000000;
    static constexpr uint32_t pwm_frequency = 25000;
    static constexpr uint32_t timer_arr = mcu_frequency / pwm_frequency / 2;  // div 2 is because we are using center aligned mode

    static constexpr int pole_pairs = 5;

    TIM_HandleTypeDef& timer;
    ADC_HandleTypeDef& adc;
    Adc_dma adc_buffer = {0};

    // at startup, the first sample is to calibrate 0A point
    int16_t current_a_zero_offset = 0;
    int16_t current_b_zero_offset = 0;
    int16_t current_c_zero_offset = 0;

    // runtime control variables
    bool pwm_on = false;
    float power = 0.0f;
    float angle = 0.0f;
    float angle_increment = 0.0f;
    Mode state = Mode::off;
    Mode old_state = Mode::off;
    StopReason stop_reason = StopReason::none;

    // calculated phase currents
    float current_a = 0.0f;
    float current_b = 0.0f;
    float current_c = 0.0f;
    float supply_voltage = 0.0f;

    float current_a_tpfilter = 0.0f;
    float current_b_tpfilter = 0.0f;
    float current_c_tpfilter = 0.0f;
    int32_t encoder_value = 0;
    int32_t encoder_offset = 2043;  // from a calibration

    Calibration calibration;
    Foc foc;

    inline void assign_pwm(float power, float angle);
    inline void assign_pwm_volt(float sa, float sb, float sc);
    inline void assign_stop();
    inline void setup_timer();
    inline void setup_adc();
    inline float calculate_supply_voltage(uint16_t sample);
    inline float calculate_adc_current(int16_t sample);
    inline void adc_calculate();
    inline void calibrate_adc();
    inline void run_calibration_step();
    inline void run_power_control();
    inline void run_current_control();
    inline void run_position_control();
    inline void pwm_safe_assign(Vector3& v);
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
    void encoder_isr(int32_t angle_value);

    inline float get_supply_voltage() { return supply_voltage; }
    inline float get_current_a() { return current_a; }
    inline float get_current_b() { return current_b; }
    inline float get_current_c() { return current_c; }
//    inline void set_mode(Mode m) { state = m; }
//    inline Mode get_mode() const { return state; }
};
