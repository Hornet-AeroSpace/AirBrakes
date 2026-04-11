idf_component_register(SRCS "main.c"
                    INCLUDE_DIRS "." 
                    EMBED_FILES "esp32p4wifi6webpage.html"
                    REQUIRES driver esp_timer esp_driver_spi driver fatfs sd_card esp_http_server esp_wifi esp_wifi_remote esp_hosted esp_netif esp_event nvs_flash icm20948-master  matrix_functions controlled_LKF mahony apogee 
                    WHOLE_ARCHIVE
                    )
add_compile_options(-fPIE)
add_link_options(-pie)