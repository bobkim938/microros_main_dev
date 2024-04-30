#ifndef IC_SPI_H
#define IC_SPI_H

#include "driver/spi_master.h"
#include "IC_register.h"
#include "esp_log.h"

#define WRA 0x80 // Write Address (0x8+address)
#define WRD  0xA0 // Write Data (0xA+data)
#define RD0 0xC0 // Read bytes 0 + 1 (2LSB) (0xC+address)
#define RD1 0xE0 // Read Bytes 2 + 3 (2MSB) (0xE)
#define NOP 0x00 // Output read Register

// AM-IP-4k SPI WORD FORMAT = 4 bit OP-CODE + 4 bit ADDRESS + 8 bit DATA

typedef struct {
    int miso;
    int mosi;
    int sclk;
    int cs;
} IC_SPI_Config;

class IC_SPI {
    public:
        IC_SPI(IC_SPI_Config *spi_config);
        esp_err_t begin();
        esp_err_t test();
        esp_err_t read_spi(uint8_t reg, uint8_t op_code);
    private:
        spi_bus_config_t busESP = {}; // SPI bus configuration
        spi_device_interface_config_t IC_dev = {}; // SPI slave configuration
        spi_device_handle_t handle; 
        spi_transaction_t t = {};
        uint8_t sendbuf[1] = {};
        uint8_t recvbuf[1] = {};
};


#endif // IC_SPI_H


