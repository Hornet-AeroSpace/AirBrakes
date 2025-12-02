#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "angleTable.h"

uint32_t velocityCount;
uint32_t angleCount;
uint32_t minVelocity;
uint32_t vStep;
uint32_t minAngle;
uint32_t aStep;
struct dragNode* nodes = NULL;

void loadAngleTable() {
    /*data file structure
    4 bytes – velocityCount (uint32)
    4 bytes – angleCount (uint32)
    4 bytes – minimum velocity * 1000000 (uint32)
    4 bytes – velocity step * 1000000 (uint32)
    4 bytes – minimum angle * 1000000 (uint32)
    4 bytes – angle step * 1000000 (uint32)
        the rest of the file:
        repeating velocityCount amount of repeating nodes containing
            4 bytes – minDrag * 100000000 (uint32)
            4 bytes – maxDrag * 100000000 (uint32)
            2 bytes – halfMaxDiff * 100000000 (uint16)
            angleCount amount of drag values, sorted least to greatest (uint16, mapped)
            angleCount amount of angle values (uint16, indexed)
            restarting node structure after angleCount amount of pairs

        mapped means the value maps from some minimum to some maximum, EX: a mapped uint8_t with value 127, between min 10 and max 20, would map to 15
        indexed means that you get the value by multiplying by the step and adding the min, EX: minAngle = 5, angleStep = 0.5, a value of 6 would index to 8 (6 * 0.5 + 5)
    */

    if (nodes != NULL) {
        printf("Called loadAngleTable() while it was already loaded and you probably didn't mean to do that. Refreshing angle table anyways...\n");
        unloadAngleTable(); //unload the table first if it isn't already empty so you don't have a memory leak
    }

    FILE* dataFile = fopen("angleTable.dat", "rb");
    if (!dataFile) {
        fprintf(stderr, "ERROR: \"angleTable.dat\" file not found!\n");
        return;
    }

    fread(&velocityCount, 4, 1, dataFile);
    fread(&angleCount, 4, 1, dataFile);
    fread(&minVelocity, 4, 1, dataFile);
    fread(&vStep, 4, 1, dataFile);
    fread(&minAngle, 4, 1, dataFile);
    fread(&aStep, 4, 1, dataFile);

    nodes = malloc(velocityCount * sizeof(struct dragNode));
    if (nodes == NULL) {
        fprintf(stderr, "ERROR: failed to allocate memory for angle table [v]\n");
        return;
    }
    
    for (uint32_t v = 0; v < velocityCount; v++) {
        struct dragNode* currentNode = &nodes[v];
        fread(&currentNode->minDrag, 4, 1, dataFile);
        fread(&currentNode->maxDrag, 4, 1, dataFile);
        fread(&currentNode->halfMaxDiff, 2, 1, dataFile);
        
        currentNode->angleValues = malloc(angleCount * sizeof(angle_t));
        currentNode->dragValues = malloc(angleCount * sizeof(drag_t));
        if (currentNode->angleValues == NULL || currentNode->dragValues == NULL) {
            fprintf(stderr, "ERROR: failed to allocate memory for angle table [ad]\n");
            return;            
        }
        fread(&currentNode->dragValues[0], 2, angleCount, dataFile);
        fread(&currentNode->angleValues[0], 2, angleCount, dataFile);
    }
    fclose(dataFile);
}

//frees the angle table from memory
void unloadAngleTable() {
    if (nodes == NULL) return;
    for (uint32_t v = 0; v < velocityCount; v++) {
        free(nodes[v].dragValues);
        free(nodes[v].angleValues);
    }
    free(nodes);
}

float getVelocityFloat(uint32_t velocityIndex) {
    return ((float)(velocityIndex * vStep) + minVelocity) / LOW_PRECISION_SCALE;
}

float getDragFloat(drag_t dragValue, struct dragNode parentNode) {
    drag_t maxValue = (uint16_t)-1;
    return ((float)dragValue / maxValue * (parentNode.maxDrag - parentNode.minDrag) + parentNode.minDrag) / HIGH_PRECISION_SCALE;
}

float getAngleFloat(angle_t angleValue) {
    return (float)angleValue * aStep / LOW_PRECISION_SCALE + (float)minAngle / LOW_PRECISION_SCALE;
}

float getAngle(float velocity, float dragCoeffiecient) {
    if (nodes == NULL) {
        fprintf(stderr, "ERROR: call loadAngleTable() before getting an angle\n");
        return -1.0f;
    }

    //index into correct velocity (aka into nodes array)
    uint32_t scaledVelocity = velocity * LOW_PRECISION_SCALE;
    if (scaledVelocity < minVelocity) {
        fprintf(stderr, "WARNING: Tried to get airbrake angle from a velocity value smaller than the Angle Table, clamping velocity to %f\n", getVelocityFloat(0));
        scaledVelocity = minVelocity;
    }
    uint32_t vMod = (scaledVelocity - minVelocity) % vStep;
    uint32_t vClamp = scaledVelocity - vMod;
    if (vMod >= vStep/2) vClamp += vStep;
    int nodeIndex = (vClamp - minVelocity) / vStep;
    if (nodeIndex >= velocityCount) {
        fprintf(stderr, "WARNING: Tried to get airbrake angle from a velocity value bigger than the Angle Table, clamping velocity to %f\n", getVelocityFloat(velocityCount - 1));
        nodeIndex = velocityCount - 1;
    }
    struct dragNode* node = &nodes[nodeIndex];

    //binary search the drag array
    float maxDrag = (float)node->maxDrag / HIGH_PRECISION_SCALE;
    float minDrag = (float)node->minDrag / HIGH_PRECISION_SCALE;
    drag_t scaledDrag = ((dragCoeffiecient - minDrag) / (maxDrag - minDrag) * UINT16_MAX + 0.5f);
    if (dragCoeffiecient > maxDrag || dragCoeffiecient < minDrag) {
        scaledDrag = (drag_t) (dragCoeffiecient > maxDrag ? -1 : 0);
        fprintf(stderr, "WARNING: inputted drag is %s the %s drag value in the Angle Table, clamping drag value to %f\n", dragCoeffiecient > maxDrag ? "above" : "below", dragCoeffiecient > maxDrag ? "max" : "min", dragCoeffiecient > maxDrag ? maxDrag : minDrag);
    }
    int pivot = angleCount/2;
    int right = angleCount - 1;
    int left = 0;
    float angleLerp = -1.0f; //lerp between the drag at index pivot, and pivot+1
    while (right - left > 1) {
        drag_t pivotDrag = node->dragValues[pivot];
        
        if (scaledDrag > pivotDrag) {
            drag_t nextDrag = node->dragValues[pivot+1];
            if (scaledDrag <= nextDrag) {  //drag value is between the drags at index pivot & pivot+1
                if (nextDrag - pivotDrag == 0) { //protect against divide by zero
                    angleLerp = 0.0f;
                    break;
                }
                angleLerp = (scaledDrag - pivotDrag) / (float) (nextDrag - pivotDrag);
                break;
            } else {
                left = pivot;
                pivot = (right + left) / 2;
                continue;
            }
        } else {
            if (scaledDrag >= node->dragValues[pivot-1]) { //drag value is between the drags at index pivot-1 & pivot
                drag_t lastDrag = node->dragValues[pivot-1];
                pivot--;
                if (pivotDrag - lastDrag == 0) { //protect against divide by zero
                    angleLerp = 0.0f;
                    break;
                }
                angleLerp = (scaledDrag - lastDrag) / (float) (pivotDrag - lastDrag);
                break;
            }
            right = pivot;
            pivot = (right + left) / 2;
            continue;
        }
    }
    if (angleLerp == -1.0f) { //drag outside the range of values in the binary search
        if (scaledDrag < node->dragValues[0]) {
            fprintf(stderr, "WARNING: Tried to get airbrake angle from a drag value smaller than the Angle Table, clamping drag to %f\n", getDragFloat(node->dragValues[0], *node));
            return getAngleFloat(node->angleValues[0]);
        } else if (scaledDrag > node->dragValues[angleCount - 1]) {
            fprintf(stderr, "WARNING: Tried to get airbrake angle from a drag value bigger than the Angle Table, clamping drag to %f\n", getDragFloat(node->dragValues[angleCount - 1], *node));
            return getAngleFloat(node->angleValues[angleCount - 1]);
        } else fprintf(stderr, "ERROR: Binary search failed, this should never ever occur so it means Reese is bad at coding and did it wrong\n");
    }
    //lerp between adjacent indexed angles
    float lesserAngle = getAngleFloat(node->angleValues[pivot]);
    float greaterAngle = getAngleFloat(node->angleValues[pivot+ 1]);

    return (greaterAngle - lesserAngle) * angleLerp + lesserAngle;
}

void printLoadedData() {
    if (nodes == NULL) {
        printf("Data is not loaded\n");
        return;
    }

    printf("\nV ");
    for (uint32_t v = 0; v < velocityCount; v++) { //x (top row)
        printf("%6.2f ", getVelocityFloat(v));
    }
    for (uint32_t a = 0; a < angleCount; a++) { //y
        printf("\n\nD ");
        for (uint32_t v = 0; v < velocityCount; v++) { //x (drags)
            printf("%6.2f ", getDragFloat(nodes[v].dragValues[a], nodes[v]));
        }
        printf("\nA ");
        for (uint32_t v = 0; v < velocityCount; v++) { //x (angles)
            printf("%6.2f ", getAngleFloat(nodes[v].angleValues[a]));
        }

    }
    printf("\n");
}
