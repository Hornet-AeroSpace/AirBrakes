#include <stdlib.h>
#include <math.h>
#include "matrix_functions.h"
#include <HardwareSerial.h>


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

matrix m_inv(matrix m, garbage_truck* garbage_man)
{
	if(m.rows == 1 && m.cols == 1)
		return m_inv_1x1(m, garbage_man);
	if(m.rows == 2 && m.cols == 2)
		return m_inv_2x2(m, garbage_man);
	if(m.rows == 3 && m.cols == 3)
		return m_inv_3x3(m, garbage_man);
	
	matrix out;
	
	return out;
}

void m_display(matrix m)
{
  for(int i = 0; i < m.rows; i++)
  {
    for(int j = 0; j < m.cols; j++)
    {
      if(m.elems[i * m.cols + j] > 0)
        Serial.print(" ");
      Serial.print(m.elems[i * m.cols + j], 6);
    }
    Serial.print("\n");
  }
  Serial.print("\n");
}