#include <stdlib.h>
#include <math.h>
#include "matrix_functions.h"

void state_extrapolation(matrix* f_state, matrix* state, matrix* s_tran, matrix* control)
{
  garbage_truck* g_man = (garbage_truck*)malloc(sizeof(garbage_truck));
  g_man->count = 0;
  g_man->trash = (float**)malloc(8 * sizeof(float*));
  for(int i = 0; i < 8; i++)
    g_man->trash[i] = NULL;
    
  m_chuck(*f_state, g_man);
  *f_state = m_add(m_mult(*s_tran, *state, g_man), *control, g_man);
  *f_state = m_duplicate(*f_state);

  for(int i = 0; i < g_man->count; i++)
  {
    free(g_man->trash[i]);
    g_man->trash[i] = NULL;
  }
  free(g_man->trash);
  g_man->trash = NULL;
  free(g_man);
  g_man = NULL;
}

void covariance_extrapolation(matrix* f_cov, matrix* s_tran, matrix* cov, matrix* process_noise_covariance)
{
  garbage_truck* g_man = (garbage_truck*)malloc(sizeof(garbage_truck));
  g_man->count = 0;
  g_man->trash = (float**)malloc(8 * sizeof(float*));
  for(int i = 0; i < 8; i++)
    g_man->trash[i] = NULL;

  m_chuck(*f_cov, g_man);
  *f_cov = m_add(m_mult(m_mult(*s_tran, *cov, g_man), m_trans(*s_tran, g_man), g_man), *process_noise_covariance, g_man);
  *f_cov = m_duplicate(*f_cov);

  for(int i = 0; i < g_man->count; i++)
  {
    free(g_man->trash[i]);
    g_man->trash[i] = NULL;
  }
  free(g_man->trash);
  g_man->trash = NULL;
  free(g_man);
  g_man = NULL;
}

void update_gain(matrix* gain, matrix* f_cov, matrix* observation, matrix* measurement_uncertainty)
{
  garbage_truck* g_man = (garbage_truck*)malloc(sizeof(garbage_truck));
  g_man->count = 0;
  g_man->trash = (float**)malloc(32 * sizeof(float*));
  for(int i = 0; i < 32; i++)
    g_man->trash[i] = NULL;

  m_chuck(*gain, g_man);
  *gain = m_mult(m_mult(*f_cov, m_trans(*observation, g_man), g_man), m_inv(m_add(m_mult(m_mult(*observation, *f_cov, g_man), m_trans(*observation, g_man), g_man), *measurement_uncertainty, g_man), g_man), g_man); 
  *gain = m_duplicate(*gain);

  for(int i = 0; i < g_man->count; i++)
  {
    free(g_man->trash[i]);
    g_man->trash[i] = NULL;
  }
  free(g_man->trash);
  g_man->trash = NULL;
  free(g_man);
  g_man = NULL;
}

void state_update(matrix* state, matrix* f_state, matrix* gain, matrix* measurement, matrix* observation)
{
  garbage_truck* g_man = (garbage_truck*)malloc(sizeof(garbage_truck));
  g_man->count = 0;
  g_man->trash = (float**)malloc(8 * sizeof(float*));
  for(int i = 0; i < 8; i++)
    g_man->trash[i] = NULL;

  m_chuck(*state, g_man);
  *state = m_add(*f_state, m_mult(*gain, m_sub(*measurement, m_mult(*observation, *f_state, g_man), g_man), g_man), g_man);
  *state = m_duplicate(*state);

  for(int i = 0; i < g_man->count; i++)
  {
    free(g_man->trash[i]);
    g_man->trash[i] = NULL;
  }
  free(g_man->trash);
  g_man->trash = NULL;
  free(g_man);
  g_man = NULL;
}

void covariance_update(matrix* cov, matrix* identity, matrix* gain, matrix* observation, matrix* f_cov, matrix* measurement_uncertainty)
{
  garbage_truck* g_man = (garbage_truck*)malloc(sizeof(garbage_truck));
  g_man->count = 0;
  g_man->trash = (float**)malloc(32 * sizeof(float*));
  for(int i = 0; i < 32; i++)
    g_man->trash[i] = NULL;
    
  matrix pt1 = m_sub(*identity, m_mult(*gain, *observation, g_man), g_man);
  matrix pt2 = m_mult(m_mult(*gain, *measurement_uncertainty, g_man), m_trans(*gain, g_man), g_man);

  m_chuck(*cov, g_man);
  *cov = m_add(m_mult(m_mult(pt1, *f_cov, g_man), m_trans(pt1, g_man), g_man), pt2, g_man);
  *cov = m_duplicate(*cov);
  
  for(int i = 0; i < g_man->count; i++)
  {
    free(g_man->trash[i]);
    g_man->trash[i] = NULL;
  }
  free(g_man->trash);
  g_man->trash = NULL;
  free(g_man);
  g_man = NULL;
}
