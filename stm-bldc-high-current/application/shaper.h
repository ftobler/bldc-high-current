/*
 * shaper.h
 *
 *  Created on: Nov 9, 2025
 *      Author: ftobler
 */

#pragma once


class Shaper {
private:
    float max_delta;  // units per update cycle
    float target = 0.0f;
    float current = 0.0f;
public:
    Shaper(float max_delta): max_delta(max_delta) {};
    void start() { target = current; }
    float update();
    void set_target(float new_target) { target = new_target; }
};
