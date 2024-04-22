#ifndef DK42688_SPI_H
#define DK42688_SPI_H

#include "register.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include "esp_system.h"

typedef struct {
    int miso;
    int mosi;
    int sclk;
    int cs;
} DK42688_SPI_Config;

enum GyroFSR : uint8_t { // Full Scale Range gyro
    dps2000 = 0x00,
    dps1000 = 0x01,
    dps500 = 0x02,
    dps250 = 0x03,
    dps125 = 0x04,
    dps62_5 = 0x05,
    dps31_25 = 0x06,
    dps15_625 = 0x07
};
enum AccelFSR : uint8_t { // Full Scale Range accel
    g16 = 0x00,
    g8 = 0x01,
    g4 = 0x02,
    g2 = 0x03
};
enum ODR : uint8_t { // Output Data Rate
    odr32k = 0x01, // LN mode
    odr16k = 0x02, // LN mode
    odr8k = 0x03, // LN mode
    odr4k = 0x04, // LN mode
    odr2k = 0x05, // LN mode
    odr1k = 0x06, // LN mode (default)
    odr200 = 0x07, // LP, LN mode
    odr100 = 0x08, // LP, LN mode
    odr50 = 0x09, // LP, LN mode
    odr25 = 0x0A, // LP, LN mode
    odr12a5 = 0x0B, // LP, LN mode
    odr6a25 = 0x0C, // LP mode
    odr3a125 = 0x0D, // LP mode 
    odr1a5625 = 0x0E, // LP mode
    odr500 = 0x0F, // LP, LN mode
};

class DK42688_SPI {
    public:
        DK42688_SPI(DK42688_SPI_Config *spi_config);
        esp_err_t begin();
        esp_err_t reset();
        esp_err_t set_gyro_fsr(GyroFSR fsr);
        esp_err_t set_accel_fsr(AccelFSR fsr);
        esp_err_t set_accODR(ODR odr);
        esp_err_t set_gyroODR(ODR odr);
        double get_accel_x();
        double get_accel_y();
        double get_accel_z();
        double get_gyro_x();
        double get_gyro_y();
        double get_gyro_z();

    private:
        esp_err_t ret;
        spi_device_handle_t handle;
        spi_bus_config_t buscfg = {};
        spi_device_interface_config_t devcfg = {};
        spi_transaction_t t = {};
        uint8_t sendbuf[1] = {0};
        uint8_t recvbuf[1] = {0};
        float gyro_fsr = 2000.0;
        float accel_fsr = 16.0;
        
        esp_err_t read_spi(uint8_t reg);
        esp_err_t write_spi(uint8_t reg, uint8_t data, uint8_t len);
        esp_err_t who_am_i();
};


#endif // DK42688_SPI_H