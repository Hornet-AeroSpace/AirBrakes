#include "adxl375.h"

//register ids
#define ADXL_ID 0x00
#define ADXL_DATAFORMAT 0x31 //bits are: [self test, spi mode, interrupt invert, 0, 1, justify (msb, lsb), 1, 1]
//sensor data is split across two registers in twos compliment format, recommended to do a multi byte read across all sensors at the same time
//at 1600 or 3200, precision is lost, the lsb is always 0. right justified (justify == 0) this is d0 of datax0, when left justified (justify == 1) this is d3 of datax0
#define ADXL_DATAX0 0x32
#define ADXL_DATAX1 0x33
#define ADXL_DATAY0 0x34
#define ADXL_DATAY1 0x35
#define ADXL_DATAZ0 0x36
#define ADXL_DATAZ1 0x37
#define ADXL_FIFO_CTL 0x38 //bits are: [(7,6) fifo mode, trigger, (4,0) samples]
//fifo modes are: [0,0] bypass fifo, [0,1] 32 sample, only writing when not full, [1, 0] 32 sample, overwriting oldest when full, [1, 1] holds latest samples before trigger event
//samples bits are only used when fifo is enabled. fifo should be disabled for our usage
#define ADXL_XOFF 0x1E //built in offset registers, should be set based on calibration, saves processing time on the main microcontroller. twos complement
#define ADXL_YOFF 0x1F
#define ADXL_ZOFF 0x20
#define ADXL_RATE 0x2C //[0, 0, 0, low power, (3,0) rate] low power should be set to 0, rate should be set to 1111, 3200hz
#define ADXL_PWR 0x2D //should always be set to [0, 0, 0, 0, 0, 0, 0, 0] just all zeroes, disables power saving and sleep

void adxl375_initialize_device(spi_host_device_t host, spi_device_handle_t* handle, int cs_pin)
{
    spi_device_interface_config_t config = {
    .clock_speed_hz = 5000000,
    .mode = 3,
    .spics_io_num = cs_pin,
    .queue_size = 7,
    .address_bits = 0,
    .flags = 0
    };

    ESP_ERROR_CHECK(spi_bus_add_device(host, &config, handle));

    uint8_t tx[2] = {0x38, 0x00};
    spi_transaction_t transaction = {
        .length = 16,
        .tx_buffer = tx,
    };
    spi_device_polling_transmit(*handle, &transaction);

    tx[0] = 0x2C;
    tx[1] = 0x0F;
    transaction.tx_buffer = tx;
    spi_device_polling_transmit(*handle, &transaction);

    tx[0] = 0x31;
    tx[1] = 0x0F;
    transaction.tx_buffer = tx;
    spi_device_polling_transmit(*handle, &transaction);

    tx[0] = 0x2D;
    tx[1] = 0x08;
    transaction.tx_buffer = tx;
    spi_device_polling_transmit(*handle, &transaction);

    tx[0] = 0x2E;
    tx[1] = 0x80;
    transaction.tx_buffer = tx;
    spi_device_polling_transmit(*handle, &transaction);

    tx[0] = 0x2F;
    tx[1] = 0x00;
    transaction.tx_buffer = tx;
    spi_device_polling_transmit(*handle, &transaction);
}

void adxl375_read(spi_device_handle_t* handle, int16_t* output)
{
    uint8_t data[7] = {0};
    uint8_t empty[7] = {0};

    empty[0] = (0x32 | 0xC0);

    spi_transaction_t transaction = {
        .length = 7 * 8,
        .tx_buffer = empty,
        .rx_buffer = data,
    };

    spi_device_polling_transmit(*handle, &transaction);

    output[0] = (int16_t)((data[2] << 8) | data[1]);
    output[1] = (int16_t)((data[4] << 8) | data[3]);
    output[2] = (int16_t)((data[6] << 8) | data[5]);
}