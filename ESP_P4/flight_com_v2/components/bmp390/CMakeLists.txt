idf_component_register(
    SRCS "bmp3.c" "bmp390.c"               # Your source files (.c or .cpp)
    REQUIRES esp_driver_spi
    INCLUDE_DIRS ../lib      # Folder containing your .h files
)