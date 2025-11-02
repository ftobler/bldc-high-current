/*
 * as5600.c
 *
 *  Created on: Apr 19, 2024
 *      Author: ftobler
 */

#include "as5600.h"






void As5600::poll() {
	//poll AS5600 sensor
	uint8_t i2c_received[2] = {0};
	HAL_I2C_Mem_Read(&hi2c, AS5600L_ADDR, 0x0C, I2C_MEMADD_SIZE_8BIT, i2c_received, 2, 2);
	const int32_t new_angle = i2c_received[1] + ((uint16_t)i2c_received[0] << 8);

	const int32_t difference = new_angle - last_angle;
	if (difference > 2048) {
		//underflow
		overflow_value--;
	} else if (difference < -2048) {
		//overflow
		overflow_value++;
	}
	last_angle = new_angle;
	angle_value = new_angle + 4096 * overflow_value;
}
