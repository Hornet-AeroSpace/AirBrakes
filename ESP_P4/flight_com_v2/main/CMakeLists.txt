idf_component_register(SRCS "main.c" "apogee_pie.S"
                    INCLUDE_DIRS "." 
                    REQUIRES driver esp_timer esp_driver_spi driver fatfs sd_card icm20948-master  matrix_functions controlled_LKF mahony apogee
                    WHOLE_ARCHIVE
                    )
add_compile_options(-fPIE)
add_link_options(-pie)