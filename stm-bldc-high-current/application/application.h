/*
 * application.h
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#pragma once


#ifdef __cplusplus
extern "C" {
#endif


void application_init();

void application_loop();

void pwm_timer_isr();



#ifdef __cplusplus
}
#endif
