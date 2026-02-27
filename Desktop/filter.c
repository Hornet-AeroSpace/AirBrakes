#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>
#include <immintrin.h>
#include "matrix_functions.h"

const float PI = 3.141593;
const float GRAV = -9.80665;
const float deltaT = 0.02;

float* getCSVData(char* CSVFILE, int column, bool degrees, bool gees, bool feet, int* rows)
{
	FILE *filePointer = fopen(CSVFILE, "r");
	char buffer[1060];

	if(filePointer == NULL)
	{
		perror("Failed to access data file");
		exit(EXIT_FAILURE);
	}
	
	float* arr = (float*)malloc(sizeof(float));
	if(arr == NULL)
	{
		perror("Memory Allocation for CSV data array failed");
		exit(EXIT_FAILURE);
	}
	int r = 0;//counts rows
	while(fgets(buffer, sizeof(buffer), filePointer) != NULL)
	{
		int n = 0;//counts characters
		char i = buffer[n];
		int j = 0;//counts commas, counts entries/categories of data
		while(i != 0 && i != 10)
		{
			i = buffer[n];
			if(i == ',')
			{
				++j;
			}
			if(j == column)
			{
				//printf(" %d. ", j);
				if(j != 0)
					n++;
				break;
			}
			n++;
		}
		i = buffer[n];
		int m = n;
		while(buffer[m] != 0 && buffer[m] != 10 && buffer[m] != ',')
			m++;
		//printf("%d", m - n);
		char* value = (char*)malloc((m - n + 1) * sizeof(char));
		m = n;
		if(value == NULL)
			perror("Memory Allocation for value failed");
		while(true)
		{
			if(i == ',')
				break;
			i = buffer[m];
			value[m - n] = i;
			//printf("%c", value[m-n]);
			m++;
		}
		arr = (float*)realloc(arr, (r + 1) * sizeof(float));
		arr[r] = strtod(value, NULL);
		if(degrees)
			arr[r] *= PI / 180;
		if(gees)
			arr[r] *= -GRAV;
		if(feet)
			arr[r] *= 0.3048;
		//printf("%s", value);
		++r;
		free(value);
	}
	if(rows != NULL)
		*rows = r;
	return arr;
}

float kf1d(float* lastP, float* uncP, float* lastV, float* uncV, float* lastA, float* uncA, float measP, float uncM)
{
	*uncP += deltaT * deltaT * *uncV;
	float w1 = *uncP / (*uncP + uncM);
	*uncV += deltaT * deltaT * *uncA;
	float w2 = *uncV / (*uncV + uncM);
	*uncA += 0.0001;
	float w3 = *uncA / (*uncA + uncM);
	
	float predictedP = *lastP + (deltaT * *lastV) + (deltaT * deltaT * *lastA / 2);
	float predictedV = *lastV + (deltaT * *lastA);
	float predictedA = *lastA;
	
	float nextP = predictedP + w1 * (measP - predictedP);
	float nextV = (nextP - *lastP) / deltaT;
	float nextA = *lastA + w3 * ((nextV - *lastV) / deltaT - predictedA);
	
	float nextUncP = (1 - w1) * *uncP;
	float nextUncV = (1 - w2) * *uncV;
	float nextUncA = (1 - w3) * *uncA;
	
	*lastP = nextP;
	*lastV = nextV;
	*lastA = nextA;
	*uncP = nextUncP;
	*uncV = nextUncV;
	*uncA = nextUncA;
	return 0;
}

float kf1d_controlled(float* lastP, float* uncP, float* lastV, float* uncV, float* lastA, float* uncA, float measP, float uncM, float measA, float uncIMU)
{
	*uncP += deltaT * deltaT * *uncV;
	float w1 = *uncP / (*uncP + uncM);
	*uncV += deltaT * deltaT * *uncA;
	float w2 = *uncV / (*uncV + uncM);
	*uncA += 0.0001;
	float w3 = *uncA / (*uncA + uncM);
	
	float predictedP = *lastP + (deltaT * *lastV) + (deltaT * deltaT * *lastA / 2);
	float predictedV = *lastV + (deltaT * *lastA);
	float predictedA = *lastA;
	
	float nextP = predictedP + w1 * (measP - predictedP);
	float nextV = predictedV + w2 * ((measP - predictedP) / deltaT - predictedV);
	float nextA = predictedA + w3 * ((nextV - *lastV) / deltaT - predictedA);
	
	float nextUncP = (1 - w1) * *uncP;
	float nextUncV = (1 - w2) * *uncV;
	float nextUncA = (1 - w3) * *uncA;
	
	*lastP = nextP;
	*lastV = nextV;
	*lastA = nextA + uncIMU * (measA - nextA);
	//*lastA = nextA;
	*uncP = nextUncP;
	*uncV = nextUncV;
	*uncA = nextUncA;
	
	return 0;
}

float ekf(matrix* identity, matrix measurement, matrix observation, matrix measurement_variance, matrix* state, matrix* estimate_covariance, matrix* prior_covariance, matrix* gain, matrix* process_noise, matrix* observation_jacobian, matrix* noise_covariance, matrix* state_transition)
{
	garbage_truck* garbage_man = (garbage_truck*)malloc(sizeof(garbage_truck));
	garbage_man->count = 0;
	garbage_man->trash = (float**)malloc(32 * sizeof(float*));
	for(int i = 0; i < 32; i++)
		garbage_man->trash[i] = NULL;
	
	//observation matrix set
	m_modify(&observation, 0, 0, state->elems[0]);
	m_modify(&observation, 0, 2, state->elems[2]);
	
	m_chuck(*gain, garbage_man);
	m_chuck(*state, garbage_man);
	m_chuck(*estimate_covariance, garbage_man);
	m_chuck(*prior_covariance, garbage_man);
	
	//kalman gain calculation
	*gain = m_mult(m_mult(*prior_covariance, m_trans(*observation_jacobian, garbage_man), garbage_man), m_inv_1x1(m_add(m_mult(m_mult(*observation_jacobian, *prior_covariance, garbage_man), m_trans(*observation_jacobian, garbage_man), garbage_man), measurement_variance, garbage_man), garbage_man), garbage_man);
	//printf("old Kalman Gain\n");
	//m_display(*gain);
	
	//*gain = ekf_gain(prior_covariance->elems, measurement_variance.elems, garbage_man);
	//printf("new Kalman Gain\n");
	//m_display(*gain);
	
	//current state estimation x(n,n)
	*state = m_add(*state, m_mult(*gain, m_sub(measurement, observation, garbage_man), garbage_man), garbage_man);
	//printf("Current Estimate\n");
	m_display(*state);
	
	//*state = ekf_state_estimation(state->elems, gain->elems, measurement.elems, garbage_man);
	
	float out = state->elems[0];
	
	//update estimate covariance
	*estimate_covariance = m_add(m_mult(m_mult(m_sub(*identity, m_mult(*gain, *observation_jacobian, garbage_man), garbage_man), *prior_covariance, garbage_man), m_trans(m_sub(*identity, m_mult(*gain, *observation_jacobian, garbage_man), garbage_man), garbage_man), garbage_man), m_mult(m_mult(*gain, measurement_variance, garbage_man), m_trans(*gain, garbage_man), garbage_man), garbage_man);
	//printf("Estimate Covariance\n");
	//m_display(*estimate_covariance);
	
	//predict future state
	*state = m_mult(*state_transition, *state, garbage_man);
	//printf("Predict future state\n");
	//m_display(*state);
	
	//predict future covariance
	*prior_covariance = m_add(m_mult(m_mult(*state_transition, *estimate_covariance, garbage_man), m_trans(*state_transition, garbage_man), garbage_man), *process_noise, garbage_man);
	//printf("Predict future covariance\n");
	//m_display(*prior_covariance);
	
	
	*gain = m_duplicate(*gain);
	*estimate_covariance = m_duplicate(*estimate_covariance);
	*state = m_duplicate(*state);
	*prior_covariance = m_duplicate(*prior_covariance);
	
	//printf("%d\n", garbage_man->count);
	for(int i = 0; i < garbage_man->count; i++)
	{
		free(garbage_man->trash[i]);
		garbage_man->trash[i] = NULL;
	}
	free(garbage_man->trash);
	garbage_man->trash = NULL;
	free(garbage_man);
	garbage_man = NULL;
	return out;
}

int main()
{
	garbage_truck* garbage_man = (garbage_truck*)malloc(sizeof(garbage_truck));
	garbage_man->count = 0;
	garbage_man->trash = (float**)malloc(256 * sizeof(float*));
	
	//matrix initialization
	matrix identity = m_identity(3);
	matrix measurement = m_new(1, 2);
	matrix observation = m_new(1, 2);
	matrix measurement_variance = m_new(1, 2);
	matrix state = m_new(1, 3);
	matrix estimate_covariance = m_new(3, 3);
	matrix prior_covariance = m_new(3, 3);
	matrix gain = m_new(1, 3);
	matrix process_noise = m_new(3, 3);
	matrix observation_jacobian = m_new(3, 1);
	matrix noise_covariance = m_new(3, 3);
	matrix state_transition = m_new(3, 3);
	
	//reading data
	int length;
	int accLength;
	float* measurements = getCSVData("small.csv", 8, false, false, true, &length);
	float* altTime = getCSVData("small.csv", 4, false, false, false, &length);
	//float* gyroX = getCSVData("large.csv", 6, true, false, false, &accLength);
	//float* gyroY = getCSVData("large.csv", 7, true, false, false, &accLength);
	//float* gyroZ = getCSVData("large.csv", 8, true, false, false, &accLength);
	float* accX = getCSVData("large.csv", 9, false, true, false, &accLength);
	//float* accY = getCSVData("large.csv", 10, false, true, false, &accLength);
	//float* accZ = getCSVData("large.csv", 11, false, true, false, &accLength);
	float* imuTime = getCSVData("large.csv", 4, false, false, false, &accLength);
	
	for(int i = 1; i < (accLength - 37) / 10; i++)
	{
		for(int j = 1; j < 10; j++)
		{
			accX[i] += accX[i * 10 + j + 37];
		}
		accX[i] /= 10;
		accX[i] *= -1;
		accX[i] -= 9.739691;
		imuTime[i] = imuTime[i *10 + 37];
		//printf("%f. %f\n", imuTime[i], accX[i]);
	}
	accX = (float*)realloc(accX, accLength * sizeof(float));
	imuTime = (float*)realloc(imuTime, accLength * sizeof(float));
	
	//step 0
	m_modify(&state_transition, 0, 0, 1);//position is linearly related to position
	m_modify(&state_transition, 1, 1, 1);//velocity is linearly related to velocity
	m_modify(&state_transition, 2, 2, 1);//acceleration is linearly related to acceleration
	m_modify(&state_transition, 1, 0, deltaT);//velocity is deltat related to position
	m_modify(&state_transition, 2, 1, deltaT);//acceleration is deltat related to velocity
	m_modify(&state_transition, 2, 0, pow(deltaT, 2) / 2);//acceleration is 1/2 deltat ^2 related to position
	
	float acc_stddev = 5.15;
	float baro_stddev = 200;
	float imu_stddev = 0.2;
	float start_cov = 500;
	
	m_modify(&process_noise, 0, 0, pow(deltaT, 4) / 4 * acc_stddev);
	m_modify(&process_noise, 1, 0, pow(deltaT, 3) / 2 * acc_stddev);
	m_modify(&process_noise, 0, 1, pow(deltaT, 3) / 2 * acc_stddev);
	m_modify(&process_noise, 2, 0, pow(deltaT, 2) / 2 * acc_stddev);
	m_modify(&process_noise, 0, 2, pow(deltaT, 2) / 2 * acc_stddev);
	m_modify(&process_noise, 1, 1, pow(deltaT, 2) * acc_stddev);
	m_modify(&process_noise, 2, 1, deltaT * acc_stddev);
	m_modify(&process_noise, 1, 2, deltaT * acc_stddev);
	m_modify(&process_noise, 2, 2, acc_stddev);
	
	m_modify(&measurement_variance, 0, 0, baro_stddev);
	m_modify(&measurement_variance, 0, 1, imu_stddev);
	
	m_modify(&estimate_covariance, 0, 0, start_cov);
	m_modify(&estimate_covariance, 1, 1, start_cov);
	m_modify(&estimate_covariance, 2, 2, start_cov);
	
	m_modify(&state, 0, 0, measurements[1]);
	m_modify(&state, 0, 2, accX[1])	;
	
	//observation jacobian set, its just 1
	m_modify(&observation_jacobian, 0, 0, 1);
	m_modify(&observation_jacobian, 0, 2, 1);
	
	m_chuck(prior_covariance, garbage_man);
	prior_covariance = m_duplicate(m_add(m_mult(m_mult(state_transition, estimate_covariance, garbage_man), m_trans(state_transition, garbage_man), garbage_man), process_noise, garbage_man));
	
	float alt;
	
	struct timespec start, end;
	float elapsed;
	clock_gettime(CLOCK_MONOTONIC, &start);
	
	for(int i = 1; i < length; i++)
	{
		//printf("%d.\n", i);
		m_modify(&measurement, 0, 0, measurements[i]);
		m_modify(&measurement, 0, 2, accX[i]);
		ekf(&identity, measurement, observation, measurement_variance, &state, &estimate_covariance, &prior_covariance, &gain, &process_noise, &observation_jacobian, &noise_covariance, &state_transition);
		printf("alt: %f\n", state.elems[0] - measurements[58]);
	}
	
	clock_gettime(CLOCK_MONOTONIC, &end);
	elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
	printf("%f seconds\n", elapsed);
	
	printf("%d. %f\n", length, alt);
	
	m_chuck(identity, garbage_man);
	m_chuck(measurement, garbage_man);
	m_chuck(measurement_variance, garbage_man);
	m_chuck(observation, garbage_man);
	m_chuck(observation_jacobian, garbage_man);
	m_chuck(state, garbage_man);
	m_chuck(estimate_covariance, garbage_man);
	m_chuck(prior_covariance, garbage_man);
	m_chuck(gain, garbage_man);
	m_chuck(process_noise, garbage_man);
	m_chuck(noise_covariance, garbage_man);
	m_chuck(state_transition, garbage_man);
	
	for(int i = 0; i < garbage_man->count; i++)
	{
		free(garbage_man->trash[i]);
	}
	free(garbage_man->trash);
	free(garbage_man);
	
	free(measurements);
	
	return 0;
}