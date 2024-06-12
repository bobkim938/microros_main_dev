#ifndef DI_DO_SPI_H
#define DI_DO_SPI_H

#include "esp_err.h"
#include "driver/spi_master.h"

#define CID 0x0

typedef struct {
    int miso;
    int mosi;
    int sclk;
    int cs;
} DI_DO_SPI_config;

enum DI_DO_REG : uint8_t {
    CHAN_STATUS = 0x0, // Current value of each of the eight input channels {STATUS[7:0]}
    DBNC_MODE0 = 0x1, // Mode control bits for the first four channel debounce filters organized as: {md_ch3[1:0],md_ch2[1:0],md_ch1[1:0],md_ch0[1:0]}
    DBNC_MODE1 = 0x2, // Mode control bits for the second four channel debounce filters organized as: {md_ch7[1:0],md_ch6[1:0],md_ch5[1:0],md_ch4[1:0]}
    DBNC_DLY0 = 0x3, // Delay control bits for the first four channel debounce filters organized as: {dly_ch3[1:0],dly_ch2[1:0],dly_ch1[1:0],dly_ch0[1:0]}
    DBNC_DLY1 = 0x4, // Delay control bits for the second four channel debounce filters organized as: {dly_ch7[1:0],dly_ch6[1:0],dly_ch5[1:0],dly_ch4[1:0]}
};

class DI_DO_SPI {
    public:
        DI_DO_SPI(DI_DO_SPI_config* spi_config);
    private:
        spi_device_handle_t spi;
        spi_bus_config_t buscfg;
        spi_device_interface_config_t devcfg;
        DI_DO_SPI_config* conf;
};





#endif // DI_DO_SPI_H