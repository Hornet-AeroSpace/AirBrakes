#include <stdlib.h>
#include <math.h>
#include "new_LKF.h"
/*
matrices are defined as (rxc), rows by columns
state(2x1)
s_tran(2x2)
control(2x1)
cov(2x2)
measurement_uncertainty(1x1)
process_noise_covariance(2x2)
gain(2x1)
measurement(1x1)
identity(2x2)

i've optimized certain things according with these assumptions:
observation is always [1, 0]
identity is always [1, 0, 0, 1]
*/


void new_state_extrapolation(float* state, float* s_tran, float* control)
{
  //*state = m_add(m_mult(*s_tran, *state), *control);
  float temp[2];
  //s_tran(2x2) * state (2x1)
  temp[0] = state[0] * s_tran[0] + state[1] * s_tran[1];
  temp[1] = state[0] * s_tran[2] + state[1] * s_tran[3];
  //ans (2x1) + control (2x1)
  temp[0] = temp[0] + control[0];
  temp[1] = temp[1] + control[1];
  //state = ans 
  state[0] = temp[0];
  state[1] = temp[1];
}

void new_covariance_extrapolation(float* s_tran, float* cov, float* process_noise_covariance)
{
  //*cov = m_add(m_mult(m_mult(*s_tran, *cov), m_trans(*s_tran)), *process_noise_covariance);
  float temp_a[4];
  float temp_b[4];
  //s_tran (2x2) * cov (2x2)
  temp_a[0] = s_tran[0] * cov[0] + s_tran[1] * cov[2];
  temp_a[1] = s_tran[0] * cov[1] + s_tran[1] * cov[3];
  temp_a[2] = s_tran[2] * cov[0] + s_tran[3] * cov[2];
  temp_a[3] = s_tran[2] * cov[1] + s_tran[3] * cov[3];
  //s_tran^T (2x2)
  temp_b[0] = s_tran[0];
  temp_b[1] = s_tran[2];
  temp_b[2] = s_tran[1];
  temp_b[3] = s_tran[3];
  //ans1 (2x2) * ans2 (2x2)
  cov[0] = temp_a[0] * temp_b[0] + temp_a[1] * temp_b[2];
  cov[1] = temp_a[0] * temp_b[1] + temp_a[1] * temp_b[3];
  cov[2] = temp_a[2] * temp_b[0] + temp_a[3] * temp_b[2];
  cov[3] = temp_a[2] * temp_b[1] + temp_a[3] * temp_b[3];
  //ans (2x2) + process_noise_covariance (2x2)
  cov[0] = cov[0] + process_noise_covariance[0];
  cov[1] = cov[1] + process_noise_covariance[1];
  cov[2] = cov[2] + process_noise_covariance[2];
  cov[3] = cov[3] + process_noise_covariance[3];
  //cov = ans
}

void new_update_gain(float* gain, float* cov, float measurement_uncertainty)
{
  //*gain = m_mult(m_mult(*cov, m_trans(*observation)), m_inv(m_add(m_mult(m_mult(*observation, *cov), m_trans(*observation)), *measurement_uncertainty))); 
  float temp_a;
  float temp_b;
  //observation^T (1x2) = ans1(2x1)
  //for this implementation, the transpose of a 2x1 or a 1x2 matrix is identical to the unmodified matrix
  //observation[1] is always 0 because we aren't measuring velocity so we can skip a couple steps and truncate some operations
  //observation(1x2) * cov(2x2) = ans2(1x2)
  temp_a = cov[0];
  //ans2(1x2) * ans1(2x1)
  temp_b = temp_a;
  //ans(1x1) + measurement_uncertainty(1x1)
  temp_b = temp_b + measurement_uncertainty;
  //ans^-1(1x1) = ans3(1x1)
  temp_b = 1.0f / temp_b;
  //cov(2x2) * ans1(2x1)
  gain[0] = cov[0];
  gain[1] = cov[2];
  //ans3(1x1) * ans(2x1)
  gain[0] = gain[0] * temp_b;
  gain[1] = gain[1] * temp_b;
}

void new_state_update(float* state, float* gain, float measurement)
{
  float temp_a;
  float temp_b[2];
  //*state = m_add(*state, m_mult(*gain, m_sub(*measurement, m_mult(*observation, *state))));
  //observation(1x2) * state(2x1) = ans(1x1)
  //measurement(1x1) - ans(1x1)
  temp_a = measurement - state[0];
  //gain(2x1) * ans(1x1)
  temp_b[0] = gain[0] * temp_a;
  temp_b[1] = gain[1] * temp_a;
  //state + ans
  state[0] = state[0] + temp_b[0];
  state[1] = state[1] + temp_b[1];
}

void new_covariance_update(float* cov, float* gain, float measurement_uncertainty)
{
  float temp_a[4];
  float temp_b[4];
  float temp_c[4];
  float temp_d;
  //matrix pt1 = m_sub(*identity, m_mult(*gain, *observation));
  temp_a[0] = 1.0f - gain[0];
  temp_a[1] = 0;
  temp_a[2] = -gain[1];
  temp_a[3] = 1;
  //matrix pt2 = m_mult(m_mult(*gain, *measurement_uncertainty), m_trans(*gain));
  temp_b[0] = gain[0] * measurement_uncertainty;
  temp_b[3] = gain[1] * measurement_uncertainty;

  temp_b[0] = temp_b[0] * gain[0];
  temp_b[1] = temp_b[0] * gain[1];
  temp_b[2] = temp_b[3] * gain[0];
  temp_b[3] = temp_b[3] * gain[1];
  //*cov = m_add(m_mult(m_mult(pt1, *cov), m_trans(pt1)), pt2);

  temp_c[0] = cov[0] * temp_a[0];
  temp_c[1] = cov[1] * temp_a[0];
  temp_c[2] = cov[0] * temp_a[2] + cov[2];
  temp_c[3] = cov[1] * temp_a[2] + cov[3];

  temp_d = temp_a[1];
  temp_a[1] = temp_a[2];
  temp_a[2] = temp_d;

  cov[0] = temp_c[0] * temp_a[0] + temp_c[1] * temp_a[2];
  cov[1] = temp_c[0] * temp_a[1] + temp_c[1] * temp_a[3];
  cov[2] = temp_c[2] * temp_a[0] + temp_c[3] * temp_a[2];
  cov[3] = temp_c[2] * temp_a[1] + temp_c[3] * temp_a[3];

  cov[0] = cov[0] + temp_b[0];
  cov[1] = cov[1] + temp_b[1];
  cov[2] = cov[2] + temp_b[2];
  cov[3] = cov[3] + temp_b[3];
}

