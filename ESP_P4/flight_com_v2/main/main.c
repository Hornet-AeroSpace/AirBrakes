#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_timer.h"
#include "driver/spi_master.h"
#include "esp_clk_tree.h"
#include "esp_vfs_fat.h"
#include "driver/sdmmc_host.h"
#include "sd_test_io.h"

#include "icm20948.h"
#include "icm20948_spi.h"
#include "sdmmc_cmd.h"

#include "matrix_functions.h"
#include "mahony.h"
#include "controlled_LKF.h"
#include "apogee.h"



//spi2
#define HSPI_MOSI 0
#define HSPI_MISO 0
#define HSPI_CLK 0
//spi3
#define VSPI_MOSI 0
#define VSPI_MISO 0
#define VSPI_CLK 0
//sdmmc
#define SDMMC_WIDTH 4


void add_pie(int16_t *x, int16_t *y, int16_t *z, int n)
{
    asm volatile(
        " csrwi 0x7F2, 0x3\n"
        " fence\n"
        " fence.i\n"  
        ::: "memory"
    );
    asm volatile(
        " addi sp, sp, -32 \n"
        " sw x31, 28(sp) \n"
        " sw x30, 24(sp) \n"
        " sw x29, 20(sp) \n"
        " sw x28, 16(sp) \n"
        " sw x27, 12(sp) \n"
        
        " mv x31, %0 \n"
        " mv x30, %1 \n"
        " mv x29, %2 \n"
        " mv x28, %3 \n"
        " li x27, 0 \n"
        " loop:\n"
        " bge x27, x28, exit \n"
        
        " esp.vld.128.ip q0, x31, 16 \n"
        " esp.vld.128.ip q1, x31, 16 \n"
        " esp.vld.128.ip q2, x31, 16 \n"
        " esp.vld.128.ip q3, x31, 16 \n"
        " esp.vld.128.ip q4, x30, 16 \n"
        " esp.vld.128.ip q5, x30, 16 \n"
        " esp.vld.128.ip q6, x30, 16 \n"
        " esp.vld.128.ip q7, x30, 16 \n"
        
        " esp.vadd.s16 q0, q0, q4 \n"
        " esp.vadd.s16 q1, q1, q5 \n"
        " esp.vadd.s16 q2, q2, q6 \n"
        " esp.vadd.s16 q3, q3, q7 \n"
        
        " esp.vst.128.ip q0, x29, 16 \n"
        " esp.vst.128.ip q1, x29, 16 \n"
        " esp.vst.128.ip q2, x29, 16 \n"
        " esp.vst.128.ip q3, x29, 16 \n"
        
        " addi x27, x27, 16\n"
        " j loop \n"
        " exit:\n"
        
        " lw x31, 28(sp) \n"
        " lw x30, 24(sp) \n"
        " lw x29, 20(sp) \n"
        " lw x28, 16(sp) \n"
        " lw x27, 12(sp) \n"
        " addi sp, sp, 32 \n"
        
        :: "r" (x), "r" (y) , "r" (z), "r" (n)
    );
}

extern float simulate_apogee_vector(float* startVels, float* startHeights, float* outHeights, int* timeSteps, float* coeffTable, int* coeffOffsets, float area, float mass);

void app_main(void)
{
    /*
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = 43;
    slot_config.cmd = 44;
    slot_config.d0 = 39;
    slot_config.d1 = 40;
    slot_config.d2 = 41;
    slot_config.d3 = 42;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    */

    printf("main start\n");

    float startHeight = 400;
    float vx = 0;
    float vy = 150;
    float vz = 0;
    float startTemp = 273;
    int steps = 0;

    int start = esp_timer_get_time();

    float alt = simulateApogeeRungeKutta(startHeight, vx, vz, vy, startTemp, &steps);

    int runtime = esp_timer_get_time() - start;
    printf("%.4f meters, %.4f seconds, %.4f ms runtime\n", alt, (float)steps * 0.16, (float)runtime / 1000.0);

    float startHeights[4] __attribute__((aligned(16))) = {0, 0, 0, 0};
    float startVels[4] __attribute__((aligned(16))) = {0, 0, 0, 0};
    float outHeights[4] __attribute__((aligned(16))) = {0, 0, 0, 0};
    int timeSteps[4] __attribute__((aligned(16))) = {1, 1, 1, 1};
    float coeffTable[4] __attribute__((aligned(16))) = {1, 1, 1, 1};
    int coeffOffsets[4] __attribute__((aligned(16))) = {0, 0, 0, 0};
    float area = 0.005027;
    float mass = 1.451;

    float asmOut = simulate_apogee_vector(startVels, startHeights, outHeights, timeSteps, coeffTable, coeffOffsets, area, mass);

    printf("asmOut == %f\n", asmOut);
    printf("startHeight, startVel, outHeight, timeSteps\n");
    for(int i = 0; i < 4; i++)
    {
        printf("%d. %f, %f, %f, %d\n", i, startHeights[i], startVels[i], outHeights[i], timeSteps[i]);
    }
    while(true)
    {
        vTaskDelay(100);
        printf("main loop\n");
    }
}
