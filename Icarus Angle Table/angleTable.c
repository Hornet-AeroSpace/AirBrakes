#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "angleTable.h"

//esp32 p4; c6
//windspeed & fin angle at specific increments, drag coeff arbitrary


//size_t vCount = 2001; //amount of velocity data (collumns)
//const size_t aCount = 902; //amount of angle data (rows)


int main() {
    generateDemoData(5, 5);
    preprocessCSV("demo_data.csv");
}

void loadAngleTable() {
    /*data file structure
    4 bytes – velocityCount (uint32)
    4 bytes – angleCount (uint32)
    4 bytes – min angle * 1000000 (uint32)
    4 bytes – max angle * 1000000 (uint32)
    4 bytes – minimum velocity * 1000000 (uint32)
    4 bytes – velocity step * 1000000 (uint32)
        the rest of the file:
        repeating velocityCount amount of repeating nodes containing
            4 bytes – minDrag * 100000000 (uint32)
            4 bytes – maxDrag * 100000000 (uint32)
            2 bytes – halfMaxDiff * 100000000 (uint16)
            drag values every first 2 bytes, sorted least to greatest (uint16, mapped)
            angle values corresponding to the preceding drag value every second 2 bytes (uint16, mapped)
            restarting node structure after angleCount amount of pairs

        mapped means the value maps from some minimum to some maximum, EX: a mapped uint8_t with value 127, between min 10 and max 20, would map to 15
    */

    FILE* dataFile = fopen("angleTable.dat", "rb");
    
    
}

void printLoadedData() {

}


//AHHHH also verify that everything sorted right with a big print and a small dataset

// float* loadData() {
//     float test[10][10];



//     return &test;
//}