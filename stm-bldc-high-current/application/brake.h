/*
 * brake.h
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#pragma once


#include "stm32_hal.h"


class Brake {
private:
    TIM_HandleTypeDef& timer;
public:
    Brake(TIM_HandleTypeDef& timer): timer(timer) {}
    void init();
    void update(float voltage);
};
