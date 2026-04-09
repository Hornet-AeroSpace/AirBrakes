#ifndef MATRIX_FUNCTIONS_H
#define MATRIX_FUNCTIONS_H

#include <stdio.h>


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

matrix m_sub(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_trans(matrix m, garbage_truck* garbage_man);

matrix m_mult(matrix m1, matrix m2, garbage_truck* garbage_man);

matrix m_inv_1x1(matrix m, garbage_truck* garbage_man);

matrix m_inv_2x2(matrix m, garbage_truck* garbage_man);

matrix m_inv_3x3(matrix m, garbage_truck* garbage_man);

matrix m_inv(matrix m, garbage_truck* garbage_man);

void m_display(matrix m);

#endif //MATRIX_FUNCTIONS_H