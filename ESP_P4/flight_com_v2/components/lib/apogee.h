#include <stdbool.h>

float calcDen(float height, float temp);

float calcAirRes(float denTot, float vel);

float lookupCoeff(float vel);

float calcGravAlt(float height);

float simulateApogeeRungeKutta(float startHeight, float vx, float vz, float vy, float startTemp, int* steps);