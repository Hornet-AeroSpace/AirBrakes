#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>
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
#define STARTINGALT 0 //starting altitude
#define MASS 0.359 //rocket mass when motor its out of fuel in kg
#define ERROR 0.001 //accepted difference between predicted apogee and desired apogee when calculating needed CDA

#define TIMESTEP 0.16 //timesteps of simulation in seconds

const float COEFFTABLE[2][13] = {{0, 85, 125, 147, 188, 214, 232, 244, 259, 266, 270, 282, 293.5}, {0.55, 0.56, 0.57, 0.58, 0.59, 0.62, 0.63, 0.64, 0.65, 0.66, 0.67, 0.68, 0.69}};

const char CSVFILE[] = "large.csv";

float CURRANGLE; //current airbrake angle

float* getCSVData(int column, int length, bool degrees, bool gees)
{
	FILE *filePointer = fopen(CSVFILE, "r");
	char buffer[1060];

	if(filePointer == NULL)
	{
		perror("Failed to access data file");
		exit(EXIT_FAILURE);
	}
	
	float* arr = (float*)malloc(length * sizeof(float));
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
			++n;
			if(i == ',')
			{
				++j;
			}
			if(j == column)
			{
				//printf(" %d. ", j);
				break;
			}
		}
		i = buffer[n];
		int m = n;
		while(buffer[m] != 0 && buffer[m] != 10 && buffer[m] != ',')
			m++;
		//printf("%d", m - n);
		char* value = (char*)malloc((m - n) * sizeof(char));
		m = n;
		if(value == NULL)
			perror("Memory Allocation for value failed");
		while(i != ',')
		{
			i = buffer[m];
			value[m - n] = i;
			//printf("%c", value[m-n]);
			m++;
		}
		arr[r] = strtod(value, NULL);
		if(degrees)
			arr[r] *= PI / 180;
		if(gees)
			arr[r] *= -GRAV;
		//printf("%s", value);
		++r;
		free(value);
	}
	return arr;
}

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
	float height = startHeight, temp = startTemp, density = calcDen(height, temp), ax, az, ay, airRes;
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

float simulateApogeeDiffeq(float height, float v, float temp, int* steps, float deltaT)
{
	float start_temp = temp;
	float start_height = height;
	*steps = 0;
	float start_v = v;
	float g = -calcGravAlt(height);
	float a = calcDen(height, temp) * AREA * lookupCoeff(v) / MASS / 2;
	float b = sqrt(g / a);
	float c = atan(start_v / sqrt(g / a)) / (a * sqrt(g / a));
	//printf("a: %f, b: %f, c: %f\n", a, b, c);
	while(v > 0)
	{
		temp = start_temp - (height - start_height) * LAPSERATE;
		g = -calcGravAlt(height);
		a = calcDen(height, temp) * AREA * lookupCoeff(v) / MASS / 2;
		b = sqrt(g / a);
		c = atan(start_v / sqrt(g / a)) / (a * sqrt(g / a));
		//printf("a: %f, b: %f, c: %f\n", a, b, c);
		float t = deltaT * *steps;
		v = b * tan(a * b * (c - t));
		//printf("v = %f\n", v);
		height += v * deltaT;
		++(*steps);
	}
	return height;
}

float simulateApogeeDiffeqRungeKutta(float height, float v, float temp, int* steps, float deltaT)
{
	*steps = 0;
	float start_temp = temp;
	float start_height = height;
	float start_v = v;
	float v1, v2, v3, v4;
	float h1, h2, h3;
	float g = -calcGravAlt(height);
	float a = calcDen(height, temp) * AREA * lookupCoeff(v) / MASS / 2;
	float b = sqrt(g / a);
	float c = atan(start_v / sqrt(g / a)) / (a * sqrt(g / a));
	float t;
	while(v > 0)
	{
		temp = start_temp - (height - start_height) * LAPSERATE;
		g = -calcGravAlt(height);
		a = calcDen(height, temp) * AREA * lookupCoeff(v) / MASS / 2;
		b = sqrt(g / a);
		c = atan(start_v / sqrt(g / a)) / (a * sqrt(g / a));
		t = deltaT * *steps;
		v1 = b * tan(a * b * (c - t));
		h1 = height + v1 * deltaT / 2;
		
		//printf("h1 %f v1 %f\n", h1, v1);
		g = -calcGravAlt(h1);
		a = calcDen(h1, temp) * AREA * lookupCoeff(v1) / MASS / 2;
		b = sqrt(g / a);
		c = atan(start_v / sqrt(g / a)) / (a * sqrt(g / a));
		t = deltaT * *steps + deltaT / 2;
		v2 = b * tan(a * b * (c - t));
		h2 = height + v2 * deltaT / 2;
		
		//printf("h2 %f v2 %f\n", h2, v2);
		g = -calcGravAlt(h2);
		a = calcDen(h2, temp) * AREA * lookupCoeff(v2) / MASS / 2;
		b = sqrt(g / a);
		c = atan(start_v / sqrt(g / a)) / (a * sqrt(g / a));
		t = deltaT * *steps + deltaT / 2;
		v3 = b * tan(a * b * (c - t));
		h3 = height + v3 * deltaT;
		
		//printf("h3 %f v3 %f\n", h3, v3);
		g = -calcGravAlt(h3);
		a = calcDen(h3, temp) * AREA * lookupCoeff(v3) / MASS / 2;
		b = sqrt(g / a);
		c = atan(start_v / sqrt(g / a)) / (a * sqrt(g / a));
		t = deltaT * (*steps + 1);
		v4 = b * tan(a * b * (c - t));
		
		v = (v1 + 2 * v2 + 2 * v3 + v4) / 6;
		
		height += v * deltaT;
		
		//printf("h %f v4 %f\n", height, v);
		++(*steps);
	}
	
	return height;
}

int main()
{
	struct timespec start, end;
	float elapsed;
	clock_gettime(CLOCK_MONOTONIC, &start);
	int steps;
	//(double startHeight, float lastVelx, float lastVelz, float lastVely, float startTemp)
	//float apogee = simulateApogee(3432.5 + STARTINGALT, 0, 0, 612.7, 305.5); //predicted apogee
	//printf("apogee: %f meters\n", apogee);
	float apogee = 0;
	float real = 762.63;
	apogee = simulateApogeeRungeKutta(55.94, 0.01, 0, 272.14, 287.7, &steps);
	printf("runge kutta apogee: %f meters in %f seconds\n", apogee, (float)steps * TIMESTEP);
	clock_gettime(CLOCK_MONOTONIC, &end);
	elapsed = (end.tv_sec - start.tv_sec) + (end.tv_nsec - start.tv_nsec) / 1e9;
	printf("%f milliseconds\n", elapsed * 1000);
	return 0;
	
}
