#ifndef IC_SPI_H
#define IC_SPI_H

#include "driver/spi_master.h"
#include "IC_register.h"
#include "esp_log.h"

#define WRA 0b1000 // Write Address (0x8+address) 0b1000
#define WRD 0b1010 // Write Data (0xA+data) 0b1010
#define RD0 0b1100 // Read bytes 0 + 1 (2LSB) (0xC+address) 0b1100
#define RD1 0b1110 // Read Bytes 2 + 3 (2MSB) (0xE) 0b1110
#define NOP 0b0000 // Output read Register 
#define HWA 0b0000 // hardware address (default)

// AM-IP-4k SPI WORD FORMAT = 4 bit OP-CODE + 4 bit ADDRESS + 8 bit DATA

static const char *TAG1 = "IC_SPI";

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
        esp_err_t read_spi(uint8_t reg, uint8_t length, uint8_t op_code = 0);
    private:
        spi_bus_config_t busESP = {}; // SPI bus configuration
        spi_device_interface_config_t IC_dev = {}; // SPI slave configuration
        spi_device_handle_t handle; 
        spi_transaction_t t = {};
        uint8_t sendbuf[1] = {};
        uint16_t recvbuf[1] = {};
};


#endif // IC_SPI_H


