/*
 * pid_controller.c
 *
 *  Created on: Oct 2, 2019
 *      Author: Ericson Joseph
 */

#include "pid_controller.h"

// PID Controller Initialization
void pidInit(PIDController_t* pid, float Kp, float Ki, float Kd, float h,
		float N, float b, float uMin, float uMax) {
	// Configuration
	pid->config.Kp = Kp;
	pid->config.Ki = Ki;
	pid->config.Kd = Kd;
	pid->config.h = h;
	pid->config.N = N;
	pid->config.b = b;
	pid->config.uMin = uMin;
	pid->config.uMax = uMax;

	// State
	pid->state.P = 0.0f;
	pid->state.I = 0.0f;
	pid->state.D = 0.0f;
	pid->state.pastD = 0.0f;
	pid->state.pastY = 0.0f;
	pid->state.futureI = 0.0f;
	pid->state.u = 0.0f;
	pid->state.u_sat = 0.0f;

}

// Calculate PID controller output u[k] and return it
float pidCalculateControllerOutput(PIDController_t* pid, float y, float r) {
	// P[k] = Kp * (b*r[k] - y[k])
	pid->state.P = pid->config.Kp * (pid->config.b * r - y);

	// D[k] = (Kd/(Kd + N*h)) * D[k-1] - (N*h*Kd/(Kd+N*h)) * (y[k]-y[k-1])
	pid->state.D = (pid->config.Kd * pid->state.pastD
			- pid->config.N * pid->config.h * pid->config.Kd
					* (y - pid->state.pastY))
			/ (pid->config.Kd + pid->config.N * pid->config.h);

	// I[k] se calculo en la enterior iteracion (en la primera se asume 0)
	pid->state.I = pid->state.futureI;

	// u[k] = P[k] + I[k] + D[k]
	pid->state.u = pid->state.P + pid->state.I + pid->state.D;

	// Apply saturation of actuator
	if (pid->state.u < pid->config.uMin) {
		pid->state.u = pid->config.uMin;
	}
	if (pid->state.u > pid->config.uMax) {
		pid->state.u = pid->config.uMax;
	}

	return pid->state.u;
}
