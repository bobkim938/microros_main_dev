#include "IMU_SPI.h"
#include "i2c_slave.h"
#include <iostream>
#include "IC_spi.h"

using namespace std;


DK42688_SPI_Config IMU_spi_config = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_10
};

IC_SPI_Config IC_spi_config = {
    .miso = GPIO_NUM_5,
    .mosi = GPIO_NUM_4,
    .sclk = GPIO_NUM_6,
    .cs = GPIO_NUM_7
};

i2c_slave_config i2c_config = {
    .sda = GPIO_NUM_18,
    .scl = GPIO_NUM_19,
    .slaveAddr = 0x0A
};

extern "C" void app_main(void)
{   
    // DK42688_SPI spi(&IMU_spi_config);
    // i2c_slave i2c(&i2c_config);
    IC_SPI IC(&IC_spi_config);
    IC.begin();
    // spi.begin();
    // spi.set_accel_fsr(AccelFSR::g16);
    // spi.set_accODR(ODR::odr1k);
    // spi.set_gyro_fsr(GyroFSR::dps2000);
    // spi.set_gyroODR(ODR::odr1k);
    for(int i = 0; i < 1000; i++) {
        IC.test();
        // uint8_t data = i2c.i2c_read();
        // ESP_LOGI("I2C", "Data received: %d", data);
        // double ax = spi.get_accel_x();
        // cout << "Accel X: " << ax << " ";
        // double ay = spi.get_accel_y();
        // cout << "Accel Y: " << ay << " ";
        // double az = spi.get_accel_z();
        // cout << "Accel Z: " << az << " ";
        // double gx = spi.get_gyro_x();
        // cout << "Gyro X: " << gx << " ";
        // double gy = spi.get_gyro_y();
        // cout << "Gyro Y: " << gy << " ";
        // double gz = spi.get_gyro_z();
        // cout << "Gyro Z: " << gz << endl;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }

}   
