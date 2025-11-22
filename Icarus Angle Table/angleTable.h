#include <stdint.h>

#define LOW_PRECISION_SCALE 1000000 //stores floats with 6 decimal precision, and as big as ~4200
#define HIGH_PRECISION_SCALE 100000000 //stores floats with high decimal precision, and as big as ~42.9 for uint32


typedef uint16_t drag_t;
typedef uint8_t angle_t;

void generateDemoData(int vSize, int aSize);
void preprocessCSV(char* filename_csv);

struct dragNode {
    drag_t* dragValues;
    angle_t* angleValues;
    uint32_t minDrag;
    uint32_t maxDrag;
    uint16_t halfMaxDiff; //half of the biggest difference of neighboring values in the list
};

