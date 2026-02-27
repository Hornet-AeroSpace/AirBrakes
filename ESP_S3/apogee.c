#include <math.h>
#include <time.h>
#include <stdbool.h>
#include "apogee.h"
/*
Sensors update every 2-20ms, ideally, calc time for sim would be less than that
*/
#define DRYAIRCONSTANT 287.058 //gas constant for dry air
#define AIRMOL 0.0289652 //molar mass of dry air
#define UNIGAS 8.31447 //uinversal gas constant
#define LAPSERATE 0.0065 //expected drop in temperature (K) per meter
#define SEALEVEL 101325 //air pressure at sea level
#define GRAV -9.80665 //standard acceleration due to gravity in m/s
#define EARTHRAD 6371008 //radius of earth in meters
#define PI 3.141593 //pi


#define AREA 0.0019634 //cross sectional area of rocket im ms2, both these may be combined with CFD
#define LOCGRAV -9.797 //accelleration due to gravity at our launch point
#define STARTINGALT 0 //starting altitude at launch point
#define MASS 0.359 //rocket mass when motor its out of fuel in kg
#define TIMESTEP 0.16 //timesteps of simulation in seconds

const float COEFFTABLE[2][13] = {{0, 85, 125, 147, 188, 214, 232, 244, 259, 266, 270, 282, 293.5}, {0.55, 0.56, 0.57, 0.58, 0.59, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68, 0.69}};

float calcDen(float height, float temp)
{//calculates air density at given height and temperature
  float pressure = SEALEVEL * powf(1.0 - 2.2557 * 0.00001 * height, 5.25588);
  return pressure * AIRMOL / (UNIGAS * temp); 
}

float calcAirRes(float denTot, float vel)
{//calculates current air resistance
 int i = 0;
  while(COEFFTABLE[0][i] < vel)
  {
    ++i;
  }
  if(i == 0)
    return -1 * denTot * COEFFTABLE[1][i] * AREA * vel * vel / 2;
  
  float ratio = (COEFFTABLE[0][i] - vel) / (COEFFTABLE[0][i] - COEFFTABLE[0][i - 1]);
  return -1 * denTot * (COEFFTABLE[1][i] * (1 - ratio) + COEFFTABLE[1][i-1] * ratio) * AREA * vel * vel / 2;
}

float lookupCoeff(float vel)
{
  int i = 0;
  while(COEFFTABLE[0][i] < vel)
  {
    ++i;
  }
  if(i == 0)
    return COEFFTABLE[1][i];
  
  float ratio = (COEFFTABLE[0][i] - vel) / (COEFFTABLE[0][i] - COEFFTABLE[0][i - 1]);
  return COEFFTABLE[1][i] * (1 - ratio) + COEFFTABLE[1][i-1] * ratio;
}

float calcGravAlt(float height)
{//gives value for gravity at the current altitude
  return LOCGRAV * (EARTHRAD / (EARTHRAD + (height - STARTINGALT)) * (EARTHRAD / (EARTHRAD + (height - STARTINGALT))));
}

float simulateApogeeRungeKutta(float startHeight, float vx, float vz, float vy, float startTemp, int* steps)
{//calculates apogee using the Runge-Kutta method of integration
  *steps = 0;
  float height = startHeight, temp = startTemp, ax, az, ay, airRes;
  float vel = powf(vx * vx + vx * vx + vy * vy, 0.5);
  float absThetaX = asinf(vx/vel);
  float absThetaZ = asinf(vz/vel);
  float theta = asinf(powf(vx * vx + vz * vz, 0.5)/vel); //gets magnitude of total angle displacement from vertical
  float v1x, v2x, v3x;
  float v1z, v2z, v3z;
  float v1y, v2y, v3y;
  float v1, v2, v3;
  float h1, h2, h3;
  
  while(vy > 0)
  {
    //v0 given, h0 given
    temp = startTemp - (height - startHeight) * LAPSERATE;
    
    //calculate h1 = h0 + v0 * stepsize/2
    h1 = height + vy * TIMESTEP / 2;
    
    //calculate a1 = a(v0, h1)
    airRes = calcAirRes(calcDen(h1, temp), vel);
    ax = airRes * sinf(absThetaX) / MASS;
    az = airRes * sinf(absThetaZ) / MASS;
    ay = airRes * cosf(theta) / MASS + calcGravAlt(h1); 
    
    //v1 = v0 + a1 * stepsize / 2
    v1x = vx + ax * TIMESTEP / 2;
    v1z = vz + az * TIMESTEP / 2;
    v1y = vy + ay * TIMESTEP / 2;
    v1 =  powf(v1x * v1x + v1z * v1z + v1y * v1y, 0.5);
    absThetaX = asinf(v1x/v1);
    absThetaZ = asinf(v1z/v1);
    theta = asinf(powf(v1x * v1x + v1z * v1z, 0.5)/v1);
    
    //calculate h2 = h0 + v1 * stepsize / 2
    h2 = height + v1y * TIMESTEP / 2;
    //calculate a2 = a(v1, h2)
    airRes = calcAirRes(calcDen(h2, temp), v1);
    ax = airRes * sinf(absThetaX) / MASS;
    az = airRes * sinf(absThetaZ) / MASS;
    ay = airRes * cosf(theta) / MASS + calcGravAlt(h2); 
    
    //v2 = v0 + a2 * stepsize / 2
    v2x = vx + ax * TIMESTEP / 2;
    v2z = vz + az * TIMESTEP / 2;
    v2y = vy + ay * TIMESTEP / 2;
    v2 =  powf(v2x * v2x + v2z * v2z + v2y * v2y, 0.5);
    absThetaX = asinf(v2x/v2);
    absThetaZ = asinf(v2z/v2);
    theta = asinf(powf(v2x * v2x + v2z * v2z, 0.5)/v2);
    
    //calculate h3 = h0 + v2 * stepsize
    h3 = height + v2y * TIMESTEP;
    //calculate a3 = a(v2, h3)
    airRes = calcAirRes(calcDen(h3, temp), v2);
    ax = airRes * sinf(absThetaX) / MASS;
    az = airRes * sinf(absThetaZ) / MASS;
    ay = airRes * cosf(theta) / MASS + calcGravAlt(h2); 
    
    //v3 = v0 + a3 * stepsize
    v3x = vx + ax * TIMESTEP;
    v3z = vz + az * TIMESTEP;
    v3y = vy + ay * TIMESTEP;
    v3 =  powf(v3x * v3x + v3z * v3z + v3y * v3y, 0.5);
    absThetaX = asinf(v3x/v3);
    absThetaZ = asinf(v3z/v3);
    theta = asinf(powf(v3x * v3x + v3z * v3z, 0.5)/v3);
    
    //h(t+1) = h0 +stepsize/6 (v0 + 2v1+ 2v2 + v3)
    height = height + (vy + 2 * v1y + 2 * v2y + v3y) * TIMESTEP / 6;
    vx = v3x;
    vz = v3z;
    vy = v3y;
    vel = v3;
    
    ++(*steps);
  }
  return height;
}
