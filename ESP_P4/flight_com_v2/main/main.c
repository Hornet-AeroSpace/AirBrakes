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

#include "sd_pwr_ctrl.h"
#include "sd_pwr_ctrl_by_on_chip_ldo.h"



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

void app_main(void)
{
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4, // Check your schematic for the correct LDO channel ID
    };

    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
    sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    sdmmc_host_deinit();
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.pwr_ctrl_handle = pwr_ctrl_handle;

    host.slot = SDMMC_HOST_SLOT_0;
    host.flags &= ~SDMMC_HOST_FLAG_DDR;
    host.max_freq_khz = 400;
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;
    slot_config.clk = 43;
    slot_config.cmd = 44;
    slot_config.d0 = 39;
    slot_config.d1 = 40;
    slot_config.d2 = 41;
    slot_config.d3 = 42;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 1,
        .allocation_unit_size = 4 * 1024
    };

    sdmmc_card_t *card;

    esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    FILE *f = fopen("/sdcard/hello.txt", "w");
    if (f == NULL) {
        printf("failed to open / create sd card file\n");
        vTaskDelay(100000);
    }
    fprintf(f, "Hello SDMMC!\n");
    fclose(f);
    esp_vfs_fat_sdcard_unmount("/sdcard", card);
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

    while(true)
    {
        vTaskDelay(100);
        printf("main loop\n");
    }
}
