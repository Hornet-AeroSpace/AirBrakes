#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include <stdbool.h>

int main()
{
	int lines = 1000;
	float noise = 0.2;
	float deltaT = 0.02;
	char* fileName = "data.txt";
	FILE* fptr;
	fptr = fopen(fileName, "w");
	float accel = 100;
	float vel = 0;
	float realPos = 0;
	float measPos;
	for(int i = 0; i < lines; i++)
	{
		accel = 100 + ((float)(rand() % 100)/ 100 - 0.5) * 10;
		measPos = realPos + ((float)(rand() % 100)/ 100 - 0.5) * noise * vel;
		char buffer[100];
		buffer[0] = '\0';
		snprintf(buffer, sizeof(buffer), "%.1f, ", measPos);
		fprintf(fptr, "%s", buffer);
		snprintf(buffer, sizeof(buffer), "%.1f, ", realPos);
		fprintf(fptr, "%s", buffer);
		snprintf(buffer, sizeof(buffer), "%.1f, ", vel);
		fprintf(fptr, "%s", buffer);
		snprintf(buffer, sizeof(buffer), "%.1f\n", accel);
		fprintf(fptr, "%s", buffer);
		realPos += vel * deltaT + accel * pow(deltaT, 2) / 2;
		vel += accel * deltaT;
	}
	return 0;
}