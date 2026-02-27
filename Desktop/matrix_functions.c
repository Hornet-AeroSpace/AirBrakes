	/*
ABG:
simple arithmetic

KF:
m_add(A, B)
m_sub(A, B)
m_mult(A, B)
m_trans(A)
m_inv(A)


EKF:
partial derivative function
jacobian matrix assembly
*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <immintrin.h>
#include <time.h>
#include "matrix_functions.h"

matrix m_new(int cols, int rows)
{
	matrix output;
	output.elems = (float*)malloc(cols * rows * sizeof(float));
	output.cols = cols;
	output.rows = rows;
	for(int i = 0; i < cols * rows; i++)
	{
		output.elems[i] = 0;
	}
	return output;
}

matrix m_identity(int size)
{
	matrix output;
	output.elems = (float*)malloc(size * size * sizeof(float));
	output.cols = size;
	output.rows = size;
	for(int i = 0; i < size * size; i++)
	{
		output.elems[i] = 0;
	}
	for(int i = 0; i < size; i++)
	{
		output.elems[i * size + i] = 1;
	}
	return output;
}

void m_modify(matrix* object, int column, int row, float n)
{
	object->elems[row * object->cols + column] = n;
}

matrix m_duplicate(matrix m)
{
	matrix out;
	out.elems = (float*)malloc(m.cols * m.rows * sizeof(float));
	out.cols = m.cols;
	out.rows = m.rows;
	for(int i = 0; i < m.cols * m.rows; i++)
	{
		out.elems[i] = m.elems[i];
	}
	return out;
}

void m_chuck(matrix m, garbage_truck* garbage_man)
{
	garbage_man->trash[garbage_man->count++] = m.elems;
}

matrix m_add(matrix m1, matrix m2, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(m1.cols * m1.rows * sizeof(float));
	out.cols = m1.cols;
	out.rows = m1.rows;
	for(int i = 0; i < out.cols * out.rows; i++)
	{
		out.elems[i] = m1.elems[i] + m2.elems[i];
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_add_128(matrix m1, matrix m2, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(m1.cols * m1.rows * sizeof(float));
	out.cols = m1.cols;
	out.rows = m1.rows;
	int loops = m1.cols * m1.rows / 4;
	int rem = m1.cols * m1.rows % 4;
	__m128 a, b, result;
	for(int i = 0; i < loops; i++)
	{
		a = _mm_load_ps(&m1.elems[i * 4]);
		b = _mm_load_ps(&m2.elems[i * 4]);
		result = _mm_add_ps(a, b);
		_mm_store_ps(&out.elems[i * 4], result);
	}
	for(int i = 0; i < rem; i++)
	{
		out.elems[loops * 4 + i] = m1.elems[loops * 4 + i] + m2.elems[loops * 4 + i];
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_add_256(matrix m1, matrix m2, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(m1.cols * m1.rows * sizeof(float));
	out.cols = m1.cols;
	out.rows = m1.rows;
	int loops = m1.cols * m1.rows / 8;
	int rem = m1.cols * m1.rows % 8;
	__m256 a, b, result;
	for(int i = 0; i < loops; i++)
	{
		a = _mm256_load_ps(&m1.elems[i * 8]);
		b = _mm256_load_ps(&m2.elems[i * 8]);
		result = _mm256_add_ps(a, b);
		_mm256_store_ps(&out.elems[i * 8], result);
	}
	for(int i = 0; i < rem; i++)
	{
		out.elems[loops * 8 + i] = m1.elems[loops * 8 + i] + m2.elems[loops * 8 + i];
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_sub(matrix m1, matrix m2, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(m1.cols * m1.rows * sizeof(float));
	out.cols = m1.cols;
	out.rows = m1.rows;
	for(int i = 0; i < out.cols * out.rows; i++)
	{
		out.elems[i] = m1.elems[i] - m2.elems[i];
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_sub_128(matrix m1, matrix m2, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(m1.cols * m1.rows * sizeof(float));
	out.cols = m1.cols;
	out.rows = m1.rows;
	int loops = m1.cols * m1.rows / 4;
	int rem = m1.cols * m1.rows % 4;
	__m128 a, b, result;
	for(int i = 0; i < loops; i++)
	{
		a = _mm_load_ps(&m1.elems[i * 4]);
		b = _mm_load_ps(&m2.elems[i * 4]);
		result = _mm_sub_ps(a, b);
		_mm_store_ps(&out.elems[i * 4], result);
	}
	for(int i = 0; i < rem; i++)
	{
		out.elems[loops * 4 + i] = m1.elems[loops * 4 + i] - m2.elems[loops * 4 + i];
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_sub_256(matrix m1, matrix m2, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(m1.cols * m1.rows * sizeof(float));
	out.cols = m1.cols;
	out.rows = m1.rows;
	int loops = m1.cols * m1.rows / 8;
	int rem = m1.cols * m1.rows % 8;
	__m256 a, b, result;
	for(int i = 0; i < loops; i++)
	{
		a = _mm256_load_ps(&m1.elems[i * 8]);
		b = _mm256_load_ps(&m2.elems[i * 8]);
		result = _mm256_sub_ps(a, b);
		_mm256_store_ps(&out.elems[i * 8], result);
	}
	for(int i = 0; i < rem; i++)
	{
		out.elems[loops * 8 + i] = m1.elems[loops * 8 + i] - m2.elems[loops * 8 + i];
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_trans(matrix m, garbage_truck* garbage_man )
{
	matrix out;
	out.cols = m.rows;
	out.rows = m.cols;
	out.elems = (float*)malloc(out.cols * out.rows * sizeof(float));
	for(int i = 0; i < out.rows; i++)
	{
		for(int j = 0; j < out.cols; j++)
		{
			out.elems[i * out.cols + j] = m.elems[j * m.cols + i];
		}
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_mult(matrix m1, matrix m2, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(m1.rows * m2.cols * sizeof(float));
	out.cols = m2.cols;
	out.rows = m1.rows;
	for(int i = 0; i < out.rows; i++)
	{
		for(int j = 0; j < out.cols; j++)
		{
			out.elems[i * out.cols + j] = 0;
			for(int k = 0; k < m1.cols; k++)
			{
				out.elems[i * out.cols + j] += m1.elems[i * m1.cols + k] * m2.elems[k * m2.cols + j];
			}
		}
	}
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix m_inv_1x1(matrix m, garbage_truck* garbage_man)
{
	matrix out;
	out.elems = (float*)malloc(1 * sizeof(float));
	out.cols = 1;
	out.rows = 1;
	garbage_man->trash[garbage_man->count++] = out.elems;
	
	out.elems[0] = 1 / m.elems[0];
	
	return out;
}

matrix m_inv_2x2(matrix m, garbage_truck* garbage_man)
{
	matrix out;
	out.elems = (float*)malloc(4 * sizeof(float));
	out.cols = 2;
	out.rows = 2;
	garbage_man->trash[garbage_man->count++] = out.elems;
	
	float det = m.elems[0] * m.elems[3] - m.elems[1] * m.elems[2];
	
	out.elems[0] = m.elems[3] / det;
	out.elems[1] = m.elems[1] * -1 / det;
	out.elems[2] = m.elems[2] * -1 / det;
	out.elems[3] = m.elems[0] / det;
	
	return out;
}

matrix m_inv_3x3(matrix m, garbage_truck* garbage_man )
{
	matrix out;
	out.elems = (float*)malloc(9 * sizeof(float));
	out.cols = 3;
	out.rows = 3;
	garbage_man->trash[garbage_man->count++] = out.elems;
	
	float det = (m.elems[0] * (m.elems[4] * m.elems[8] - m.elems[7] * m.elems[5])) - (m.elems[3] * (m.elems[1] * m.elems[8] - m.elems[7] * m.elems[2])) + (m.elems[6] * (m.elems[1] * m.elems[5] - m.elems[4] * m.elems[2]));
	
	out.elems[0] = (m.elems[4] * m.elems[8] - m.elems[7] * m.elems[5]) / det;
	out.elems[1] = (m.elems[6] * m.elems[5] - m.elems[3] * m.elems[8]) / det;
	out.elems[2] = (m.elems[3] * m.elems[7] - m.elems[6] * m.elems[4]) / det;
	out.elems[3] = (m.elems[7] * m.elems[2] - m.elems[1] * m.elems[8]) / det;
	out.elems[4] = (m.elems[0] * m.elems[8] - m.elems[6] * m.elems[2]) / det;
	out.elems[5] = (m.elems[6] * m.elems[1] - m.elems[0] * m.elems[7]) / det;
	out.elems[6] = (m.elems[1] * m.elems[5] - m.elems[4] * m.elems[2]) / det;
	out.elems[7] = (m.elems[3] * m.elems[2] - m.elems[0] * m.elems[5]) / det;
	out.elems[8] = (m.elems[0] * m.elems[4] - m.elems[3] * m.elems[1]) / det;
	
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix ekf_gain(float* prior_covariance,float* measurement_variance, garbage_truck* garbage_man)
{
	matrix out;
	out.elems = (float*)malloc(3 * sizeof(float));
	out.cols = 1;
	out.rows = 3;
	
	float temp[3];
	float single_temp;

	temp[0] = prior_covariance[0];
	temp[1] = prior_covariance[3];
	temp[2] = prior_covariance[6];

	single_temp = temp[0];

	single_temp += measurement_variance[0];

	single_temp = 1 / single_temp;
	
	temp[0] = prior_covariance[0];
	temp[1] = prior_covariance[1];
	temp[2] = prior_covariance[2];
	
	out.elems[0] = temp[0] * single_temp;
	out.elems[1] = temp[1] * single_temp;
	out.elems[2] = temp[2] * single_temp;
	
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix ekf_state_estimation(float* state, float* gain, float* measurement, garbage_truck* garbage_man)
{
	matrix out;
	out.elems = (float*)malloc(3 * sizeof(float));
	out.cols = 1;
	out.rows = 3;
	
	float temp = measurement[0] - state[0];

	out.elems[0] = gain[0] * temp + state[0];
	out.elems[1] = gain[1] * temp + state[1];
	out.elems[2] = gain[2] * temp + state[2];
	
	garbage_man->trash[garbage_man->count++] = out.elems;
	return out;
}

matrix estimate_covaraince(float* identity, float* gain, float* observation_jacobian, float* prior_covariance, float* measurement_variance, garbage_truck* garbage_man)
{
	
}

matrix future_covariance()
{
	
}

float covariance(float* x, float* y, int size)
{	
	float cov = 0;
	float mean1 = 0;
	float mean2 = 0;
	
	for(int i = 0; i < size; i++)
	{
		mean1 += x[i];
		mean2 += y[i];
	}
	
	mean1 /= size;
	mean2 /= size;
	
	for(int i = 0; i < size; i++)
		cov += x[i] * y[i];
	
	cov /= size - 1;

	cov -= mean1 * mean2 * size/ (size - 1);

	return cov;
}

void m_display(matrix m)
{
	for(int i = 0; i < m.rows; i++)
	{
		for(int j = 0; j < m.cols; j++)
		{
			printf("%.8f, ", m.elems[i * m.cols + j]);
		}
		printf("\n");
	}
	printf("\n");
}