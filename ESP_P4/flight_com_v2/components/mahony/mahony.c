#include <stdlib.h>
#include <math.h>
#include "mahony.h"

#define GTOM 9.80665
#define SERIAL_PORT Serial

void quat_integrate(float* quat, float wx, float wy, float wz, float deltaT)
{
	float dq0 = (-quat[1] * wx - quat[2] * wy - quat[3] * wz) / 2.0;
    float dq1 = ( quat[0] * wx + quat[2] * wz - quat[3] * wy) / 2.0;
    float dq2 = ( quat[0] * wy - quat[1] * wz + quat[3] * wx) / 2.0;
    float dq3 = ( quat[0] * wz + quat[1] * wy - quat[2] * wx) / 2.0;
	
	quat[0] += dq0 * deltaT;
    quat[1] += dq1 * deltaT;
    quat[2] += dq2 * deltaT;
    quat[3] += dq3 * deltaT;
}

void quat_normalize(float* quat)
{
	float factor =  sqrtf(powf(quat[0], 2) + powf(quat[1], 2) + powf(quat[2], 2) + powf(quat[3], 2));
	
	if(factor == 0)
		return;
	
	float inv = 1.0 / factor;
	
	quat[0] *= inv;
	quat[1] *= inv;
	quat[2] *= inv;
	quat[3] *= inv;
}

void mahony_gyro_update(mahony_state* state, float deltaT)
{
	float wx = state->gyr[0] + state->mag_kp * state->mag_err[0] + state->acc_kp * state->acc_err[0] + state->acc_ki * state->eint_acc[0] + state->mag_ki * state->eint_mag[0];	
	float wy = state->gyr[1] + state->mag_kp * state->mag_err[1] + state->acc_kp * state->acc_err[1] + state->acc_ki * state->eint_acc[1] + state->mag_ki * state->eint_mag[1];	
	float wz = state->gyr[2] + state->mag_kp * state->mag_err[2] + state->acc_kp * state->acc_err[2] + state->acc_ki * state->eint_acc[2] + state->mag_ki * state->eint_mag[2];	
	
	quat_integrate(state->quat, wx, wy, wz, deltaT);
	quat_normalize(state->quat);
}

void mahony_acc_update(mahony_state* state, float deltaT)
{
	float net_acc = sqrtf(powf(state->acc[0], 2) + powf(state->acc[1], 2) + powf(state->acc[2], 2));
	
	float tx = state->acc[0] / net_acc;
	float ty = state->acc[1] / net_acc;
	float tz = state->acc[2] / net_acc;
	
	float vx = 2.0 * (state->quat[1] * state->quat[3] - state->quat[0] * state->quat[2]);
    float vy = 2.0 * (state->quat[0] * state->quat[1] + state->quat[2] * state->quat[3]);
    float vz = state->quat[0] * state->quat[0] - state->quat[1] * state->quat[1] - state->quat[2] * state->quat[2] + state->quat[3] * state->quat[3];
	
	state->acc_err[0] = (ty * vz - tz * vy);
	state->acc_err[1] = (tz * vx - tx * vz);
	state->acc_err[2] = (tx * vy - ty * vx);
	
	state->eint_acc[0] += state->acc_err[0] * deltaT;
    state->eint_acc[1] += state->acc_err[1] * deltaT;
    state->eint_acc[2] += state->acc_err[2] * deltaT;
}

void mahony_mag_update(mahony_state* state, float deltaT)
{
	float q0 = state->quat[0];
	float q1 = state->quat[1];
	float q2 = state->quat[2];
	float q3 = state->quat[3];
	
	float net_mag = sqrtf(powf(state->mag[0], 2) + powf(state->mag[1], 2) + powf(state->mag[2], 2));
	
	float mx = state->mag[0] / net_mag;
	float my = state->mag[1] / net_mag;
	float mz = state->mag[2] / net_mag;
	
	float hx = mx * (q0*q0 + q1*q1 - q2*q2 - q3*q3) + 2 * my * (q1*q2 - q0*q3) + 2 * mz * (q1*q3 + q0*q2);
	float hy = 2 * mx * (q1*q2 + q0*q3) + my * (q0*q0 - q1*q1 + q2*q2 - q3*q3) + 2 * mz * (q2*q3 - q0*q1);
	float hz =  2 * mx * (q1*q3 - q0*q2) + 2 * my * (q2*q3 + q0*q1) + mz * (q0*q0 - q1*q1 - q2*q2 + q3*q3);
	
	hx = sqrtf(hx*hx + hy*hy);
	
	float vx = 2.0f * (hx * (0.5f - q2*q2 - q3*q3) + hz * (q1*q3 - q0*q2));
	float vy = 2.0f * (hx * (q1*q2 - q0*q3) + hz * (q0*q1 + q2*q3));
	float vz = 2.0f * (hx * (q0*q2 + q1*q3) + hz * (0.5f - q1*q1 - q2*q2));
	
	state->mag_err[0] = my*vz - mz*vy;
	state->mag_err[1] = mz*vx - mx*vz;
	state->mag_err[2] = mx*vy - my*vx;
	
	state->eint_mag[0] += state->mag_err[0] * deltaT;
    state->eint_mag[1] += state->mag_err[1] * deltaT;
    state->eint_mag[2] += state->mag_err[2] * deltaT;
}

void vector_from_quat(float* quat, float* acc_in, float* acc_out)
{
	float tx = 2.0 * (quat[2] * acc_in[2] - quat[3] * acc_in[1]);
    float ty = 2.0 * (quat[3] * acc_in[0] - quat[1] * acc_in[2]);
    float tz = 2.0 * (quat[1] * acc_in[1] - quat[2] * acc_in[0]);
	
	acc_out[0] = acc_in[0] + quat[0] * tx + (quat[2] * tz - quat[3] * ty);
    acc_out[1] = acc_in[1] + quat[0] * ty + (quat[3] * tx - quat[1] * tz);
    acc_out[2] = acc_in[2] + quat[0] * tz + (quat[1] * ty - quat[2] * tx);
}

void quat_to_eul(float* quat, float* eul_out)
{
	eul_out[0] = atan2(2.0 * (quat[0] * quat[1] + quat[2] * quat[3]), 1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]));
	eul_out[1] = asin(2.0 * (quat[0] * quat[2] - quat[3] * quat[1]));
	eul_out[2] = atan2(2.0 * (quat[0] * quat[3] + quat[1] * quat[2]), 1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]));
	
	eul_out[0] *= 180 / M_PI;
	eul_out[1] *= 180 / M_PI;
	eul_out[2] *= 180 / M_PI;
}