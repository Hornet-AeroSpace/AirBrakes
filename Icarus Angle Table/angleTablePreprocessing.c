//Contains code to preprocess data; this file does not need to be included on the rocket's flight computer
//The flight computer will only need the output angleTable.dat file, with angleTable.c and angleTable.h

/*input csv should be structured

      ,v1,v2,v3,v4,v5
    a1,DC,DC,DC,DC,DC
    a2,DC,DC,DC,DC,DC
    a3,DC,DC,DC,DC,DC

    where v is velocity at consistent increments & sorted least to greatest, a is angle, and DC is drag coefficient at that corresponding velocity and angle
    the first v has a leading comma beause it's a csv
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "angleTable.h"

//generates a demo test csv 
void generateDemoData(int vSize, int aSize) {
    const float velocityStart = 50.0f;
    const float angleStart = 0.0f;
    const float velocityStep = 1.0f;
    const float angleStep = 0.5;
    const float dragMultiplier = 0.0063325f;
    
    float currentVelocity = velocityStart;
    float currentAngle = angleStart;

    FILE* newFile = fopen("demo_data.csv", "w");
    fprintf(newFile, ",");
    for (int i = 0; i < vSize; i++) {
        fprintf(newFile, "%f%c", currentVelocity, i != vSize - 1 ? ',' : '\n');
        currentVelocity += velocityStep;
    }
    for (int i = 0; i < aSize; i++) {
        fprintf(newFile, "%f,", currentAngle);
        for (int j = 0; j < vSize; j++) {
            if (i == aSize - 1 && j == vSize - 1)
                 fprintf(newFile, "%f", currentAngle * (velocityStart + velocityStep * j) / 100.0f);
            else fprintf(newFile, "%f%c", currentAngle * (velocityStart + velocityStep * j) / 100.0f, j != vSize - 1 ? ',' : '\n');
        }
        currentAngle += angleStep;
    }
    fclose(newFile);
}

int parseNextFloat(FILE* file, float *floatValueParsed) {//returns 0 for ERROR, otherwise returns the delimiter found if it is one of these three: '\n' ',' 'EOF'
        char delimiter;
        char buffer[64];
        for (int i = 0; i < 64; i++) {
            int currentChar = fgetc(file);
            buffer[i] = currentChar;
            if (currentChar == ',' || currentChar == '\n' || currentChar == EOF) {//delimiter reached
                buffer[i] = '\0';
                if (sscanf(buffer, "%f", floatValueParsed) != 1) //outputs the float to whatever was passed as floatValueParsed; return 0 on error
                    return 0;
                return currentChar;
            }
        }
        return 0; //error if did not find a float in the next 64 characters
}

//generate angleTable.dat
void preprocessCSV(char* filename_csv) { //include ".csv" when passing the filename
    printf("Processing...\n");
    //=========== PART 1 ===========
    //remove all '\r' characters from the csv. On windows, newlines are represented by "\r\n", so I only have to deal with single characters for newlines
    char strippedFilename[strlen(filename_csv) + 5];
    strncpy(strippedFilename, filename_csv, strlen(filename_csv) - 4);
    strippedFilename[strlen(filename_csv) - 4] = '\0';
    strcat(strippedFilename, "_temp.csv\0");
    int gottenChar;
    FILE *originalCSV = fopen(filename_csv, "r");
    if (originalCSV == NULL) {
        fprintf(stderr, "ERROR: Failed opening csv file, is the filename correct? (including .csv at the end)\n");
        return;
    }
    
    FILE *strippedCSV = fopen(strippedFilename, "w");
    while ((gottenChar = fgetc(originalCSV)) != EOF) {
        if (gottenChar != '\r') fputc(gottenChar, strippedCSV);
    }
    fclose(originalCSV);
    fclose(strippedCSV);

    //=========== PART 2 ===========
    //open the csv that had all the '\r' characters removed
    FILE* dataFile = fopen(strippedFilename, "r");
    //read velocity values
    fseek(dataFile, 1, SEEK_SET); //skip first leading comma in the csv

    
    float v1 = -1.0f, v2 = -1.0f, vMin = -1.0f, vMax = -1.0f;
    int velocitiesFound = 0;
    //verify all velocity values are ordered least to greatest, and populate the above v-variables
    float lastVelocity = -1.0f;
    for (;;) {
        float currentVelocity;
        int delimiter = parseNextFloat(dataFile, &currentVelocity);

        if (delimiter == 0) {
            fprintf(stderr, "ERROR: Failed to parse velocity values\n");
            break;
        } else velocitiesFound++;

        if (v1 == -1.0f) {
            v1 = currentVelocity;
            vMin = currentVelocity;
        } else if (v2 == -1.0f) {
            v2 = currentVelocity;
        }
        if (lastVelocity > currentVelocity) fprintf(stderr, "ERROR: Velocity data was not sorted least to greatest\n");
        if (lastVelocity == currentVelocity) fprintf(stderr, "ERROR: duplicate entry for velocity found: %f\n", currentVelocity);

        lastVelocity = currentVelocity;

        if (delimiter == '\n') { //reached last velocity
            vMax = currentVelocity;
            break;
        } else if (delimiter == EOF) {
            fprintf(stderr, "ERROR: only velocity data (just first row of data) was detected\n");
            break;
        }
    }
    
    //=========== PART 3 ===========
    //count amount of angle values (rows)
    int anglesFound = 0, dragsFound = 0;
    long fp = ftell(dataFile);
    fseek(dataFile, fp - 1, SEEK_SET);
    bool endOfFile = false;
    while (!endOfFile) {
        int delimCheck = fgetc(dataFile);
        switch(delimCheck) {
            case ',': dragsFound++; break;
            case '\n': anglesFound++; break;
            case EOF: endOfFile = true;
        }
    }
    if (dragsFound != velocitiesFound * anglesFound) fprintf(stderr, "ERROR: drag values counted was different from velocities * angles\nCounted: %i, V*A: %i\n", dragsFound, velocitiesFound *anglesFound);
    if (dragsFound >= 200000) printf("There are %d drag values to process, one sec...\n", dragsFound);
    fseek(dataFile, fp, SEEK_SET);
    
    //=========== PART 4 ===========
    float angleMin = -1.0f, angleMax = -1.0f;
    
    //dynamically allocate this:    float drags[anglesFound][velocitiesFound]
    float **drags = malloc(anglesFound * sizeof(float*));
    for (int a = 0; a < anglesFound; a++) drags[a] = malloc(velocitiesFound * sizeof(float));

    //aggregate all drag-coeff values
    float baseAngleArray[anglesFound];
    for (int a = 0; a < anglesFound; a++) {
        float currentAngle;
        if (parseNextFloat(dataFile, &currentAngle) == 0) fprintf(stderr, "ERROR: failed parsing angle values\n");
        baseAngleArray[a] = currentAngle;
        if (a == 0) angleMin = currentAngle;
        else if (currentAngle < angleMin) angleMin = currentAngle;
        if (currentAngle > angleMax) angleMax = currentAngle;

        for (int v = 0; v < velocitiesFound; v++) {
            float currentDrag;
            if (parseNextFloat(dataFile, &currentDrag) == 0) fprintf(stderr, "ERROR: failed parsing drag values\n");
            drags[a][v] = currentDrag;
        }
    }

    
    //=========== PART 5 ===========
    //convert every float (velocities, angles, and drags) into 16 bit integers for future indexing and memory saving purposes
    //also rearrange data to be structured closer to the final angle table

    struct dragNode nodes[velocitiesFound];


    for (int v = 0; v < velocitiesFound; v++) {// for each collumn
        struct dragNode* currentNode = &nodes[v];
        currentNode->dragValues = malloc(anglesFound * sizeof(drag_t));
        currentNode->angleValues = malloc(anglesFound * sizeof(angle_t));
        float currentBaseAngleArray[anglesFound];
        for (int a = 0; a < anglesFound; a++) currentBaseAngleArray[a] = baseAngleArray[a];

        //sort drag-coeff values least to greatest, per corresponding velocity value (aka per collumn)
        for (int a = 0; a < anglesFound - 1; a++) {
            for (int i = a + 1; i < anglesFound; i++) {
                float hold;
                if (drags[a][v] > drags[i][v]) { //if the current drag is bigger than next drag, swap them
                    hold = drags[a][v];
                    drags[a][v] = drags[i][v];
                    drags[i][v] = hold;

                    //also swap angle values so they still correspond correctly
                    hold = currentBaseAngleArray[a];
                    currentBaseAngleArray[a] = currentBaseAngleArray[i];
                    currentBaseAngleArray[i] = hold;
                }
            }
        }

        //convert float drags to ints representing a lerp from dragMin to dragMax
        float dragMin = -1.0f;
        float dragMax = -1.0f;
        
        for (int a = 0; a < anglesFound; a++) { //calculate dragMin and dragMax
            float currentDrag = drags[a][v];
            
            if (a == 0) dragMin = currentDrag;
            else if (currentDrag < dragMin) dragMin = currentDrag;
            if (currentDrag > dragMax) dragMax = currentDrag;
        }
        
        currentNode->minDrag = dragMin * HIGH_PRECISION_SCALE;
        currentNode->maxDrag = dragMax * HIGH_PRECISION_SCALE;
        
        float dragRange = dragMax - dragMin;
        uint16_t maxDifference = 0;
        for (int a = 0; a < anglesFound; a++) { //finally actually do the converting drags to 16-bit int mapping from dragMin to dragMax
            float currentDrag = drags[a][v];
            
            currentNode->dragValues[a] = (drag_t) ((currentDrag - dragMin) / dragRange * UINT16_MAX + 0.5f);
            
            if (a != 0) {
                uint16_t difference = (currentNode->dragValues[a] - currentNode->dragValues[a - 1]);
                if (difference > maxDifference) maxDifference = difference;
            }

            //also convert angles to 16-bit int mappings from angleMin by angleStep
            float angleStep = (angleMax - angleMin) / (anglesFound - 1);
            currentNode->angleValues[a] = (angle_t) ((currentBaseAngleArray[a] - angleMin) / angleStep);
        }
        currentNode->halfMaxDiff = maxDifference % 2 == 1 ? maxDifference / 2 + 1 : maxDifference / 2; //half max difference, rounded up
    }
    fclose(dataFile);
    remove(strippedFilename); //delete the version of the csv that had all the '\r' characters removed at the beginning
    //=========== PART 6 ===========
    //Save data to angleTable.dat file

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
            drag values every first 2 bytes, sorted least to greatest (uint16, mapped)
            angle values corresponding to the preceding drag value every second 2 bytes (uint16, indexed)
            restarting node structure after angleCount amount of pairs

        mapped means the value maps from some minimum to some maximum, EX: a mapped uint8_t with value 127, between min 10 and max 20, would map to 15
        indexed means that you get the value by multiplying by the step and adding the min, EX: minAngle = 5, angleStep = 0.5, a value of 6 would index to 8 (6 * 0.5 + 5)
    */
    FILE *tableFile = fopen("angleTable.dat", "wb");
    uint32_t velocityCount = velocitiesFound;
    uint32_t angleCount = anglesFound;
    uint32_t minimumVelocity = vMin * 1000000UL;
    uint32_t velocityStep = (v2 - v1) * 1000000UL;
    uint32_t minAngle = angleMin * 1000000UL;
    uint32_t angleStep = (angleMax - angleMin) / (angleCount - 1) * 1000000UL;


    fwrite(&velocitiesFound, 4, 1, tableFile);
    fwrite(&angleCount, 4, 1, tableFile);
    fwrite(&minimumVelocity, 4, 1, tableFile);
    fwrite(&velocityStep, 4, 1, tableFile);
    fwrite(&minAngle, 4, 1, tableFile);
    fwrite(&angleStep, 4, 1, tableFile);


    for (int v = 0; v < velocitiesFound; v++) {

        fwrite(&nodes[v].minDrag, 4, 1, tableFile);
        fwrite(&nodes[v].maxDrag, 4, 1, tableFile);
        fwrite(&nodes[v].halfMaxDiff, 2, 1, tableFile);
        
        for (int a = 0; a < angleCount; a++) {
            fwrite(&nodes[v].dragValues[a], 2, 1, tableFile);
        }
        for (int a = 0; a < angleCount; a++) {
            fwrite(&nodes[v].angleValues[a], 2, 1, tableFile);
        }
    }
    fclose(tableFile);

    //Free memory
    for (int a = 0; a < anglesFound; a++) {
        free(drags[a]);
    }
    free(drags);
    
    for (int v = 0; v < velocitiesFound; v++) {
        free(nodes[v].angleValues);
        free(nodes[v].dragValues);
    }

    printf("Done, angleTable.dat file successfully generated!\n");
    int memoryUsage = getMemoryUsage("angleTable.dat");
    printf("This data file will take up roughly %i bytes of memory when loaded%s", memoryUsage, memoryUsage >= 32000 ? ", make sure your ESP has more than that.\n" : ".\n");
}

//returns the approximate number of bytes of memory the angle table will take up on the flight computer
int getMemoryUsage(char* dataFileName) {
    FILE* dataFile = fopen("angleTable.dat", "rb");
    if (!dataFile) {
        fprintf(stderr, "Data file \"%s\" could not be read.\n", dataFileName);
        return -1;
    }
    uint32_t velocityCount = 0;
    uint32_t angleCount = 0;
    fread(&velocityCount, 4, 1, dataFile);
    fread(&angleCount, 4, 1, dataFile);
    fclose(dataFile);
    return velocityCount * angleCount * 4 + velocityCount * 18 + 1;
}