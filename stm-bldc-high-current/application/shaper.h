/*
 * shaper.h
 *
 *  Created on: Nov 9, 2025
 *      Author: ftobler
 */

#pragma once


/**
 * Trajectory Shaper class. Transforms a step input into a smooth ramp
 * by limiting the maximum change in position per time step.
 */
class Shaper {
private:
    const float DT;         // Control loop time step (seconds)
    float max_delta;        // Maximum position change per update cycle (units/DT)
    float target = 0.0f;    // The user's final desired target position
    float current_position = 0.0f;  // The smoothed current setpoint position (theta_ref)
    float current_speed = 0.0f;  // The calculated instantaneous speed (omega_ref)

    static inline float abs(float a) {
        if (a > 0) {
            return a;
        }
        return -a;
    }

public:
    /**
     * Constructor for the Shaper.
     * max_velocity Maximum allowable velocity (rad/s)
     * dt The control loop time step (seconds)
     */
    Shaper(float max_velocity, float dt) : DT(dt) {
        // max_delta = max_velocity * dt
        // This is the max distance the position can move in one time step
        max_delta = max_velocity * DT;
    };

    /**
     * Sets the final target position for the trajectory.
     * new_target The user's desired final position (radians)
     */
    void set_target(float new_target) { target = new_target; }

    /**
     * Resets the shaper to start smoothing from the current position.
     * initial_position The system's current position to start from.
     */
    void start(float initial_position) {
        current_position = initial_position;
        target = initial_position;  // Reset target as well for safety
        current_speed = 0.0f;
    }

    /**
     * Calculates the next smooth position setpoint and the required speed.
     */
    void update();

    /**
     * Returns the instantaneous required speed (omega_ref).
     * returns: The smoothed target speed (rad/s).
     */
    float get_speed() const { return current_speed; }

    /**
     * Returns the current smoothed position
     * returns: The smoothed target position (theta_ref).
     */
    float get_position() const { return current_position; }
};
