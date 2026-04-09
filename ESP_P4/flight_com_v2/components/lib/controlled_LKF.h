#ifndef CONTROLLED_LKF_H
#define CONTROLLED_LKF_H

#include <stdlib.h>
#include <math.h>
#include "matrix_functions.h"

void state_extrapolation(matrix* f_state, matrix* state, matrix* s_tran, matrix* control);

void covariance_extrapolation(matrix* f_cov, matrix* s_tran, matrix* cov, matrix* process_noise_covariance);

void update_gain(matrix* gain, matrix* f_cov, matrix* observation, matrix* measurement_uncertainty);

void state_update(matrix* state, matrix* f_state, matrix* gain, matrix* measurement, matrix* observation);

void covariance_update(matrix* cov, matrix* identity, matrix* gain, matrix* observation, matrix* f_cov, matrix* measurement_uncertainty);

#endif //CONTROLLED_LKF_H