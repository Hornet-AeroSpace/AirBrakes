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
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "esp_event.h"
#include "nvs_flash.h" 

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

#define SD_MISO 39
#define SD_MOSI 44
#define SD_CLK 43
#define SD_CS 42

typedef enum {
    WEBSERVER = 0,
    CALIBRATE = 1, //calibration of sensors, mag, acc, gyro
    LAUNCHPAD = 2, //resting on launchpad
    BOOST = 3, //under motor power
    BURNOUT = 4, //post burnout, but before braking
    BRAKE = 5, //active airbrake control
    APOGEE = 6, //apogee has been reached, airbrakes retracting, waiting for parachute
    DESCENT = 7, //parachute deployed, controlled descent
} finite_state;

//website updatable global vars
float mass = 1.4; //rocket mass in kg, initialized to typical L1 mass
float area = 0.005; //rocket cross sectional area in m^2, initialized to typical L1 area
float pad_alt = 0; //launchpad altitude in meters
float pad_press = 101325; //launchpad air pressure in pascals
float grav = 9.80665; //local gravitational acceleration in m/s^2
float sim_timestep = 0.16; //altitude simulation timestep in seconds
finite_state rocket_state = WEBSERVER; //by default, start up the webserver to configure rocket

//lets the program see the html file and puts it in the .text section of the assembly instead of file structure
extern const uint8_t webpage_html_start[] asm("_binary_esp32p4wifi6webpage_html_start");
extern const uint8_t webpage_html_end[]   asm("_binary_esp32p4wifi6webpage_html_end");

//

esp_err_t start_ap()
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *ap_netif __attribute__((unused)) = esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_wifi_set_mode(WIFI_MODE_AP);
    static wifi_config_t ap_config = {
        .ap = {
            .ssid = "ESP32-P4WIFI6",
            .password = "aerospace",
            .channel = 1,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .max_connection = 4,
        },
    };
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &ap_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    return ESP_OK;
}

esp_err_t get_handler(httpd_req_t *req)
{
    const size_t webpage_html_size = (webpage_html_end - webpage_html_start);
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)webpage_html_start, webpage_html_size);
    return ESP_OK;
}

esp_err_t post_handler(httpd_req_t *req)
{
    char content[256];
    size_t recv_size = req->content_len;

    if (recv_size >= sizeof(content)) {
        recv_size = sizeof(content) - 1;
    }

    int ret = httpd_req_recv(req, content, recv_size);
    if (ret <= 0)
        return ESP_FAIL;
    content[ret] = '\0';

    float data[7];
    int index = 0;
    for(int i = 0; i < sizeof(data) / sizeof(float); i++)
    {
        data[i] = strtof(&(content[index]), NULL);
        printf("%.2f\n", data[i]);
        index += strlen(&(content[index]));
        index++;
    }

    mass = data[0];
    area = data[1];
    pad_alt = data[2];
    pad_press = data[3];
    grav = data[4];
    sim_timestep = data[5];
    rocket_state = (finite_state)data[6];

    return ESP_OK;
}

httpd_handle_t start_webserver()
{
    httpd_uri_t get_uri = {
    .uri      = "/",
    .method   = HTTP_GET,
    .handler  = get_handler,
    .user_ctx = NULL
    };
    httpd_uri_t update_uri = {
    .uri       = "/update",
    .method    = HTTP_POST,
    .handler   = post_handler,
    .user_ctx  = NULL
    };

    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    // Start the httpd server
    if (httpd_start(&server, &config) == ESP_OK) {
        // Register your existing GET handler (e.g., for the main page)
        // httpd_register_uri_handler(server, &index_get);
        // REGISTER THE NEW POST HANDLER HERE
        httpd_register_uri_handler(server, &get_uri);
        httpd_register_uri_handler(server, &update_uri);
        return server;
    }
    return NULL;
}

esp_err_t start_sd()
{
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = 4,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;
    sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    sd_pwr_ctrl_set_io_voltage(pwr_ctrl_handle, 3300);
    
    spi_bus_config_t bus_cfg = {
    .mosi_io_num = SD_MOSI,
    .miso_io_num = SD_MISO,
    .sclk_io_num = SD_CLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
    .max_transfer_sz = 4000,
    };

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = SPI2_HOST;
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS;
    slot_config.host_id = host.slot;

    spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);

    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };

    sdmmc_card_t *card;
    esp_vfs_fat_sdspi_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    return ESP_OK;
}

void app_main(void)
{
    printf("main start\n");
    //start access point
    start_ap();
    //start http server
    start_webserver();
    //start sd card through onboard slot
    start_sd();

    float startHeight = 400;
    float vx = 0;
    float vy = 150;
    float vz = 0;
    float startTemp = 273;
    int steps = 0;

    FILE* fp = fopen("/sdcard/p4.txt", "w");
    fprintf(fp, "Writing to SD card via SPI mode.\n");
    fclose(fp);


    while(true)
    {
        int start = esp_timer_get_time();

        float alt = simulateApogeeRungeKutta(startHeight, vx, vz, vy, startTemp, &steps);

        int runtime = esp_timer_get_time() - start;

        printf("%.4f meters, %.4f seconds, %.4f ms runtime\n", alt, (float)steps * 0.16, (float)runtime / 1000.0);
        vTaskDelay(100);
        printf("main loop\n");
    }
}
