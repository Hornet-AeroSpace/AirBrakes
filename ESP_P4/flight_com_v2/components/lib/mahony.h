#ifndef MAHONY_H
#define MAHONY_H
#include <stdlib.h>
#include <math.h>

typedef struct mahony_state{
    float quat[4];
	float acc_err[3];
	float mag_err[3];
    float gyro_bias[3];
    float eint_acc[3];
    float eint_mag[3];

	float acc[3];
    float gyr[3];
    float mag[3];
	
	float acc_kp;
	float acc_ki;
	float mag_kp;
	float mag_ki;
} mahony_state;

void quat_integrate(float* quat, float wx, float wy, float wz, float deltaT);

void quat_normalize(float* quat);

void mahony_gyro_update(mahony_state* state, float deltaT);

void mahony_acc_update(mahony_state* state, float deltaT);

void mahony_mag_update(mahony_state* state, float deltaT);

void vector_from_quat(float* quat, float* acc_in, float* acc_out);

void quat_to_eul(float* quat, float* eul_out);

#endif //MAHONY_H
