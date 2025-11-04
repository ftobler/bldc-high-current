/*
 * sine_table.h
 *
 *  Created on: Nov 1, 2025
 *      Author: ftobler
 */

#pragma once


#include <array>
#include <cmath>


constexpr size_t SINE_TABLE_SIZE = 1024;
constexpr float PI = 3.14159265359f;
constexpr float TWO_PI = 2 * PI;


extern const float sine_table[SINE_TABLE_SIZE];


inline float fast_sin(float angle) {
    const float normalized_absolute = angle / TWO_PI;
    const float normalized = normalized_absolute - std::floor(normalized_absolute);
    const size_t index = static_cast<size_t>(normalized * SINE_TABLE_SIZE);
    return sine_table[index];
}


inline float fast_cos(float angle) {
    // cosine is just sine shifted by 90°
    return fast_sin(angle + (PI / 2));
}
