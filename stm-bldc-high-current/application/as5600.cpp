/*
 * as5600.c
 *
 *  Created on: Apr 19, 2024
 *      Author: ftobler
 */

#include "as5600.h"




void As5600::init() {
//    // AS5600 CONF Register (0x07 & 0x08)
//    // Target Value: 0x0300 (SF=11, all others default)
//    // MSB (0x08) = 0x03, LSB (0x07) = 0x00
//    uint8_t config_data[2];
//    config_data[0] = 0x00; // LSB (0x07)
//    config_data[1] = 0x03; // MSB (0x08)
//
//    // Write to the 16-bit CONF register starting at the lower address 0x07.
//    // The AS5600 will automatically handle the 2-byte write to 0x07 and 0x08.
//
//    // HAL_I2C_Mem_Write(I2C_HandleTypeDef *hi2c, uint16_t DevAddress, uint16_t MemAddress, uint16_t MemAddSize, uint8_t *pData, uint16_t Size, uint32_t Timeout)
//    HAL_I2C_Mem_Write(&hi2c, AS5600L_ADDR, 0x07, I2C_MEMADD_SIZE_8BIT, config_data, 2, 100);
//
//    // Note: The datasheet recommends waiting at least 1ms after any config write.
//    HAL_Delay(2);
}


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
