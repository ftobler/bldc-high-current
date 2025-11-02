/*
 * as5600.h
 *
 *  Created on: Apr 19, 2024
 *      Author: ftobler
 */

#pragma once


#include "stm32_hal.h"


class As5600 {
private:
	static constexpr auto AS5600L_ADDR = 0x36*2;
	I2C_HandleTypeDef& hi2c;
	int32_t angle_value = 0;
	int32_t overflow_value = 0;
	int32_t last_angle = 0;
public:
	As5600(I2C_HandleTypeDef& hi2c): hi2c(hi2c) {};
	void poll();

	inline int32_t angle() { return angle_value; }
	inline int32_t overflow() { return overflow_value; }
};


