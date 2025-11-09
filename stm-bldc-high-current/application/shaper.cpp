/*
 * shaper.cpp
 *
 *  Created on: Nov 9, 2025
 *      Author: ftobler
 */

#include "shaper.h"


float Shaper::update() {
    float error = target - current;
    if (error > max_delta) {
        current = current + max_delta;
        return current;
    } else if (error < -max_delta) {
        current = current - max_delta;
        return current;
    } else {
        current = target;
        return current;
    }
}

