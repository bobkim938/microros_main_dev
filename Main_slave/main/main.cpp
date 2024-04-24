#include <driver/spi_master.h>
#include <driver/gpio.h>
#include "IMU_SPI.h"
#include "i2c_slave.h"
#include "esp_log.h"

using namespace std;

#define MOSI GPIO_NUM_11
#define MISO GPIO_NUM_13
#define SCLK GPIO_NUM_12
#define CS GPIO_NUM_10

DK42688_SPI_Config spi_config = {
    .miso = MISO,
    .mosi = MOSI,
    .sclk = SCLK,
    .cs = CS
};

i2c_slave_config i2c_config = {
    .sda = GPIO_NUM_18,
    .scl = GPIO_NUM_19,
    .slaveAddr = 0x0A
};

extern "C" void app_main(void)
{   
    DK42688_SPI spi(&spi_config);
    spi.begin();
    spi.set_accel_fsr(AccelFSR::g16);
    spi.set_accODR(ODR::odr1k);
    spi.set_gyro_fsr(GyroFSR::dps2000);
    spi.set_gyroODR(ODR::odr1k);
    while(1) {
        double ax = spi.get_accel_x();
        double ay = spi.get_accel_y();
        double az = spi.get_accel_z();
        double gx = spi.get_gyro_x();
        double gy = spi.get_gyro_y();
        double gz = spi.get_gyro_z();
        double imu[6] = {ax, ay, az, gx, gy, gz};
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}   
