#include "IMU_SPI.h"
#include "i2c_slave.h"
#include <iostream>
#include <fstream>
#include "IC_spi.h"

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
    // i2c_slave i2c(&i2c_config);
    spi.begin();
    spi.set_accel_fsr(AccelFSR::g16);
    spi.set_accODR(ODR::odr1k);
    spi.set_gyro_fsr(GyroFSR::dps2000);
    spi.set_gyroODR(ODR::odr1k);
    fstream output;
    output.open("IMU_data.txt");
    for(int i = 0; i < 1000; i++) {
        // uint8_t data = i2c.i2c_read();
        // ESP_LOGI("I2C", "Data received: %d", data);
        // double ax = spi.get_accel_x();
        // double ay = spi.get_accel_y();
        // double az = spi.get_accel_z();
        // double gx = spi.get_gyro_x();
        // double gy = spi.get_gyro_y();
        // double gz = spi.get_gyro_z();
        int16_t ax0 = spi.get_ax0();
        double ax = spi.get_accel_x();
        cout << "Ax0: " << ax0 << " ";
        int16_t ax1 = spi.get_ax1();
        cout  << "Ax1: " << ax1 << " ";
        int16_t ay0 = spi.get_ay0();
        double ay = spi.get_accel_y();
        cout  << "Ay0: " << ay0 << " ";
        int16_t ay1 = spi.get_ay1();
        cout  << "Ay1: " << ay1 << " ";
        int16_t az0 = spi.get_az0();
        double az = spi.get_accel_z();
        cout  << "Az0: " << az0 << " ";
        int16_t az1 = spi.get_az1();
        cout  << "Az1: " << az1 << " ";
        int16_t gx0 = spi.get_gx0();
        double gx = spi.get_gyro_x();
        cout  << "Gx0: " << gx0 << " ";
        int16_t gx1 = spi.get_gx1();
        cout  << "Gx1: " << gx1 << " ";
        int16_t gy0 = spi.get_gy0();
        double gy = spi.get_gyro_y();
        cout  << "Gy0: " << gy0 << " ";
        int16_t gy1 = spi.get_gy1();
        cout  << "Gy1: " << gy1 << " ";
        int16_t gz0 = spi.get_gz0();
        double gz = spi.get_gyro_z();
        cout  << "Gz0: " << gz0 << " ";
        int16_t gz1 = spi.get_gz1();
        cout  << "Gz1: " << gz1 << " ";
        cout << endl;

        // cout << "Ax: " << ax << " ";
        // cout << "Ay: " << ay << " ";
        // cout << "Az: " << az << " ";
        // cout << "Gx: " << gx << " ";
        // cout << "Gy: " << gy << " ";
        // cout << "Gz: " << gz << " ";
        // cout << endl;
    }
    cout << "Data written to IMU_data.txt" << endl;

}   
