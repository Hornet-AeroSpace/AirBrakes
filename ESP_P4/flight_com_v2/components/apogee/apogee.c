#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <stdbool.h>
#include "apogee.h"

#define DRYAIRCONSTANT 287.058 //gas constant for dry air
#define AIRMOL 0.0289652 //molar mass of dry air
#define UNIGAS 8.31447 //uinversal gas constant
#define LAPSERATE 0.0065 //expected drop in temperature (K) per meter
#define SEALEVEL 101325 //air pressure at sea level
#define GRAV -9.80665 //standard acceleration due to gravity in m/s
#define EARTHRAD 6371008 //radius of earth in meters
#define PI 3.141593 //pi
#define AREA 0.005027//cross sectional area of rocket im ms2, both these may be combined with CFD
#define LOCGRAV -9.797 //accelleration due to gravity at our launch point
#define STARTINGALT 1025 //starting altitude
//#define MASS 1.16151 //rocket mass when motor its out of fuel in kg
#define MASS 1.452
#define ERROR 0.001 //accepted difference between predicted apogee and desired apogee when calculating needed CDA

#define TIMESTEP 0.16 //timesteps of simulation in seconds

#define GTOM 9.80665

const float COEFFTABLE[2][8] = {
{0,    81,   115,  143,  165,  186,  207,  220},
{0.56, 0.57, 0.58, 0.59, 0.6, 0.61, 0.62, 0.63}};

float CURRANGLE; //current airbrake angle

float calcDen(float height, float temp)
{//calculates air density at given height and temperature
	float pressure = SEALEVEL * pow(1.0 - 2.2557 * 0.00001 * height, 5.25588);
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
	float vel = pow(vx * vx + vx * vx + vy * vy, 0.5);
	float absThetaX = asin(vx/vel);
	float absThetaZ = asin(vz/vel);
	float theta = asin(pow(vx * vx + vz * vz, 0.5)/vel); //gets magnitude of total angle displacement from vertical
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
		ax = airRes * sin(absThetaX) / MASS;
		az = airRes * sin(absThetaZ) / MASS;
		ay = airRes * cos(theta) / MASS + calcGravAlt(h1); 
		//v1 = v0 + a1 * stepsize / 2
		v1x = vx + ax * TIMESTEP / 2;
		v1z = vz + az * TIMESTEP / 2;
		v1y = vy + ay * TIMESTEP / 2;
		v1 =  pow(v1x * v1x + v1z * v1z + v1y * v1y, 0.5);
		absThetaX = asin(v1x/v1);
		absThetaZ = asin(v1z/v1);
		theta = asin(pow(v1x * v1x + v1z * v1z, 0.5)/v1);
		//calculate h2 = h0 + v1 * stepsize / 2
		h2 = height + v1y * TIMESTEP / 2;
		//calculate a2 = a(v1, h2)
		airRes = calcAirRes(calcDen(h2, temp), v1);
		ax = airRes * sin(absThetaX) / MASS;
		az = airRes * sin(absThetaZ) / MASS;
		ay = airRes * cos(theta) / MASS + calcGravAlt(h2); 
		//v2 = v0 + a2 * stepsize / 2
		v2x = vx + ax * TIMESTEP / 2;
		v2z = vz + az * TIMESTEP / 2;
		v2y = vy + ay * TIMESTEP / 2;
		v2 =  pow(v2x * v2x + v2z * v2z + v2y * v2y, 0.5);
		absThetaX = asin(v2x/v2);
		absThetaZ = asin(v2z/v2);
		theta = asin(pow(v2x * v2x + v2z * v2z, 0.5)/v2);
		//calculate h3 = h0 + v2 * stepsize
		h3 = height + v2y * TIMESTEP;
		//calculate a3 = a(v2, h3)
		airRes = calcAirRes(calcDen(h3, temp), v2);
		ax = airRes * sin(absThetaX) / MASS;
		az = airRes * sin(absThetaZ) / MASS;
		ay = airRes * cos(theta) / MASS + calcGravAlt(h2); 
		//v3 = v0 + a3 * stepsize
		v3x = vx + ax * TIMESTEP;
		v3z = vz + az * TIMESTEP;
		v3y = vy + ay * TIMESTEP;
		v3 =  pow(v3x * v3x + v3z * v3z + v3y * v3y, 0.5);
		absThetaX = asin(v3x/v3);
		absThetaZ = asin(v3z/v3);
		theta = asin(pow(v3x * v3x + v3z * v3z, 0.5)/v3);
		//h(t+1) = h0 +stepsize/6 (v0 + 2v1+ 2v2 + v3)
		height = height + (vy + 2 * v1y + 2 * v2y + v3y) * TIMESTEP / 6;
		vx = v3x;
		vz = v3z;
		vy = v3y;
		vel = v3;
		/*
		if(steps % (int)roundf(1/TIMESTEP) == 0)
		{
			//printf("\t\t\t\tabsThetaX %.2f absThetaZ %.2f theta %.2f\n", absThetaX, absThetaZ, theta);
			//printf("%d.\t after calc accel:\t vX %.2f vZ %.2f vY %.2f V %.2f aX %.2f aZ %.2f aY %.2f H %f T %.2f \n", steps, vx, vz, vy, vel, ax, az, ay, height, temp);
		}
		*/
		++(*steps);
	}
	//printf("%d.\t after calc accel:\t vX %.2f vZ %.2f vY %.2f V %.2f aX %.2f aZ %.2f aY %.2f H %f T %.2f \n", steps, vx, vz, vy, vel, ax, az, ay, height, temp);
	//printf("%.2f seconds to apogee\n", steps * TIMESTEP);
	return height;
}