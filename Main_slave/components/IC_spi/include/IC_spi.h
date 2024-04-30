#ifndef IC_SPI_H
#define IC_SPI_H

#include "driver/spi_master.h"

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
    private:
        spi_bus_config_t busESP = {}; // SPI bus configuration
        spi_device_interface_config_t IC_dev = {}; // SPI slave configuration
        spi_device_handle_t handle; 
};


#endif // IC_SPI_H


