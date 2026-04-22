idf_component_register(
    SRCS "adxl375.c"               # Your source files (.c or .cpp)
    REQUIRES esp_driver_spi
    INCLUDE_DIRS ../lib      # Folder containing your .h files
)