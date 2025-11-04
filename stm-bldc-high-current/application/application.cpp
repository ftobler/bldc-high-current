/*
 * application.cpp
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#include "application.h"
#include "stm32_hal.h"
#include "gpio_low_level.h"
#include "main.h"
#include "motor.h"
#include "brake.h"
#include "as5600.h"


/*
 * Note about ADC/timer setup
 * TIM1 ARR is configured through the motor module
 * TIM3 is slaved off of TIM1 (TIM1 resets TIM3 on ARR, so TIM3 never reaches its ARR)
 * ADC is triggered on TIM1 Capture Compare 6 when ARR is at 100%
 */
extern ADC_HandleTypeDef hadc1;  // main.c
extern TIM_HandleTypeDef htim1;  // main.c (pwm)
extern TIM_HandleTypeDef htim3;  // main.c (brake)
extern I2C_HandleTypeDef hi2c2;  // main.c (eeprom)
extern I2C_HandleTypeDef hi2c3;  // main.c (encoder)


static void update_led();


static Motor motor(
        htim1,
        hadc1
);
static Brake brake (htim3);
static As5600 encoder(hi2c3);

static volatile bool encoder_poll = false;



void application_init() {
    brake.init();
    motor.init();
}


void application_loop() {
    update_led();
    if (encoder_poll) {
        // encoder task gets executed in 4kHz interval
        encoder.poll();
        encoder_poll = false;
    }
}


void update_led() {
    constexpr uint32_t num_led = 8;
    constexpr uint32_t cycle_ticks = 500 / num_led;  // ticks per LED step
    const uint32_t t = uwTick;

    // determine which LED should be on
    uint32_t phase = (t / cycle_ticks) % num_led;

    // turn off all LEDs first
    gpio_ll_write(LED1, 0);
    gpio_ll_write(LED2, 0);
    gpio_ll_write(LED3, 0);
    gpio_ll_write(LED4, 0);
    gpio_ll_write(LED5, 0);
    gpio_ll_write(LED6, 0);
    gpio_ll_write(LED7, 0);
    gpio_ll_write(LED8, 0);

    // turn on the current LED
    switch (phase) {
        case 0: gpio_ll_write(LED1, 1); break;
        case 1: gpio_ll_write(LED2, 1); break;
        case 2: gpio_ll_write(LED3, 1); break;
        case 3: gpio_ll_write(LED4, 1); break;
        case 4: gpio_ll_write(LED5, 1); break;
        case 5: gpio_ll_write(LED6, 1); break;
        case 6: gpio_ll_write(LED7, 1); break;
        case 7: gpio_ll_write(LED8, 1); break;
    }
}


/**
 * fast task with 24kHz
 */
void pwm_timer_isr() {
    motor.timer_isr();
    brake.update(motor.get_supply_voltage());

    // clock breakdown for encoder
    static constexpr uint16_t count_reload = 5;
    static uint16_t count = count_reload;
    if (count) {
        count--;
    } else {
        count = count_reload;
        encoder_poll = true;  // indicate that encoder neds to be polled

        const int32_t angle_value = encoder.angle();
        motor.encoder_isr(angle_value);  // do the part of the control loops and dsp that runs on encoder frequency
    }
}
