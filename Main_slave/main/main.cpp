#include <iostream>
#include "IMU_SPI.h"
#include "i2c_slave.h"
#include "IC_spi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <std_msgs/msg/int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

using namespace std;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

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
    .sda = GPIO_NUM_18, // 15
    .scl = GPIO_NUM_19, // 16
    .slaveAddr = 0x0A
};

extern "C" void app_main(void)
{   
    DK42688_SPI spi(&IMU_spi_config);
    // i2c_slave i2c(&i2c_config);
    // IC_SPI IC(&IC_spi_config);
    // IC.begin();
    spi.begin();
    spi.set_accel_fsr(AccelFSR::g16);
    spi.set_accODR(ODR::odr1k);
    spi.set_gyro_fsr(GyroFSR::dps2000);
    spi.set_gyroODR(ODR::odr1k);
    for(int i = 0; i < 10; i++) {
        // IC.test();
        // uint8_t data = i2c.i2c_read();
        // ESP_LOGI("I2C", "Data received: %d", data);
        double ax = spi.get_accel_x();
        cout << "Accel X: " << ax << " ";
        double ay = spi.get_accel_y();
        cout << "Accel Y: " << ay << " ";
        double az = spi.get_accel_z();
        cout << "Accel Z: " << az << " ";
        double gx = spi.get_gyro_x();
        cout << "Gyro X: " << gx << " ";
        double gy = spi.get_gyro_y();
        cout << "Gyro Y: " << gy << " ";
        double gz = spi.get_gyro_z();
        cout << "Gyro Z: " << gz << endl;
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}   
