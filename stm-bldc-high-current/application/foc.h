/*
 * foc.h
 *
 *  Created on: Nov 2, 2025
 *      Author: ftobler
 */

#pragma once


struct Vector3 {
	float a;
	float b;
	float c;
};


class Foc {
private:

	// --- control gains (tune later) ---
	float kp_d = 0.005f;
	float ki_d = 0.0005f;
	float kp_q = 0.005f;
	float ki_q = 0.0005f;

	// --- PI integrators ---
	float integrator_d = 0.0f;
	float integrator_q = 0.0f;

	// --- targets ---
	float target_id = 0.0f;
	float target_iq = 0.0f;

	// --- results (for debugging / visibility) ---
	float id      = 0.0f;
	float iq      = 0.0f;

	inline void clarke_transform(float a, float b, float c, float& alpha, float& beta);
	inline void inv_clarke_transform(float alpha, float beta, float& a, float& b, float& c);
	inline void park_transform(float alpha, float beta, float angle, float& d, float& q);
	inline void inv_park_transform(float d, float q, float angle, float& alpha, float& beta);
public:
	void start();
	Vector3 update(float current_a, float current_b, float current_c, float angle);
};
