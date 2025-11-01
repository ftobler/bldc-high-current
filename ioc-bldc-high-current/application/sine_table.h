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
    // angle is in radians, wrap to [0, 2π)
    angle -= TWO_PI * std::floor(angle / TWO_PI);
    float index_f = angle * (SINE_TABLE_SIZE / TWO_PI);
    size_t index = static_cast<size_t>(index_f);
    float frac = index_f - static_cast<float>(index);

    float a = sine_table[index];
    float b = sine_table[(index + 1) % SINE_TABLE_SIZE];
    return a + (b - a) * frac;  // linear interpolation
}


inline float fast_cos(float angle) {
    // cosine is just sine shifted by 90°
    return fast_sin(angle + (PI / 2));
}
