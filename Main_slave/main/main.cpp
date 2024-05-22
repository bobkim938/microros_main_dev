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
#include <sensor_msgs/msg/imu.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

using namespace std;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
sensor_msgs__msg__Imu imu_msg = {};
static size_t uart_port = UART_NUM_0; // UART port for Micro_ROS

DK42688_SPI_Config IMU_spi_config = {
    .miso = GPIO_NUM_5,
    .mosi = GPIO_NUM_4,
    .sclk = GPIO_NUM_6,
    .cs = GPIO_NUM_7
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

void publish_imuData() {
    rcl_ret_t rc;
    rc = rcl_publish(&publisher, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
        printf("Failed to publish IMU data");
    }
}

void node_init() {
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	// create node
	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "ESP32", "", &support));

	// create publisher
	RCCHECK(rclc_publisher_init_default(
		&publisher,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"IMU_data"));
    rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

extern "C" void app_main(void)
{   

    IC_SPI ic(&IC_spi_config);
    // gpio_set_direction(GPIO_NUM_15, GPIO_MODE_OUTPUT);
    gpio_set_direction(GPIO_NUM_8, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_7, 1);
    vTaskDelay(10);
    gpio_set_level(GPIO_NUM_8, 1);
    vTaskDelay(5000 /portTICK_PERIOD_MS);
    ic.begin();
    ic.test();
    // while(1) {
    //     gpio_set_level(GPIO_NUM_15, 1);
    // }
    // while(1) {
    //     ic.test();
    // }

//     #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
// 	rmw_uros_set_custom_transport(
// 		true,
// 		(void *) &uart_port,
// 		esp32_serial_open,
// 		esp32_serial_close,
// 		esp32_serial_write,
// 		esp32_serial_read
// 	);
//     #else`
//     #error micro-ROS transports misconfigured
//     #endif  // RMW_UXRCE_TRANSPORT_CUSTOM
//     node_init();
//     DK42688_SPI spi(&IMU_spi_config);
//     spi.begin();
//     spi.set_accel_fsr(AccelFSR::g16);
//     spi.set_accODR(ODR::odr1k);
//     spi.set_gyro_fsr(GyroFSR::dps2000);
//     spi.set_gyroODR(ODR::odr1k);
//     while(1) {
//         double ax = spi.get_accel_x();
//         imu_msg.linear_acceleration.x = ax; 
//         cout << "Accel X: " << ax << " ";
//         double ay = spi.get_accel_y();
//         imu_msg.linear_acceleration.y = ay;
//         cout << "Accel Y: " << ay << " ";
//         double az = spi.get_accel_z();
//         imu_msg.linear_acceleration.z = az;
//         cout << "Accel Z: " << az << " ";
//         double gx = spi.get_gyro_x();
//         imu_msg.angular_velocity.x = gx;
//         cout << "Gyro X: " << gx << " ";
//         double gy = spi.get_gyro_y();
//         imu_msg.angular_velocity.y = gy;
//         cout << "Gyro Y: " << gy << " ";
//         double gz = spi.get_gyro_z();
//         imu_msg.angular_velocity.z = gz;
//         cout << "Gyro Z: " << gz << endl;
//         vTaskDelay(100/portTICK_PERIOD_MS);
//         publish_imuData();
//     }
}   
