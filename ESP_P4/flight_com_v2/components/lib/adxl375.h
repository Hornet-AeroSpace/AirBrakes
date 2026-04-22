#include "driver/spi_master.h"
void adxl375_initialize_device(spi_host_device_t host, spi_device_handle_t* handle, int cs_pin);

void adxl375_read(spi_device_handle_t* handle, int16_t* output);