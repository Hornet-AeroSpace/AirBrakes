#ifndef MATRIX_FUNCTIONS_H
#define MATRIX_FUNCTIONS_H

typedef struct
{
	float* elems;
	int cols;
	int rows;
}matrix;

typedef struct
{
	float** trash;
	int count;
}garbage_truck;

matrix m_new(int columns, int rows);

matrix m_identity(int size);

void m_modify(matrix* object, int column, int row, float n);

matrix m_duplicate(matrix m);

void m_chuck(matrix m, garbage_truck* garbage_man);

matrix m_add(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_add_128(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_add_256(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_sub(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_sub_128(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_sub_256(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_trans(matrix m, garbage_truck* garbage_man);

matrix m_mult(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_inv_1x1(matrix m, garbage_truck* garbage_man);

matrix m_inv_2x2(matrix m, garbage_truck* garbage_man);

matrix m_inv_3x3(matrix m, garbage_truck* garbage_man);

matrix ekf_gain(float* prior_covariance, float* measurement_variance, garbage_truck* garbage_man);

matrix ekf_state_estimation(float* state, float* gain, float* measurement, garbage_truck* garbage_man);

float covariance(float* x, float* y, int size);

void m_display(matrix m);

/*
expectation of random variable x = mean of x = sum(x * probability(x))
E(a) = a when a is constant
E(aX) = aE(x) when a is constant
E(a+X) = a + E(x)
E(X+Y) = E(X) + E(Y) both random
E(XY) = E(X)E(Y) both random

variance of constants is 0
V(a) = 0
V(X) = E(X^2) - mean(x)^2
COV(X, Y) = E(XY) - mean(x)mean(y)
if x and y are independent, COV(X, Y) should be 0
V(aX) = a^2 * V(x)
V (X + Y) = V(X) + V(Y) + 2COV(X, Y)
*/


#endif //MATRIX_FUNCTIONS_H