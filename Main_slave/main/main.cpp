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

DK42688_SPI spi(&IMU_spi_config);

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
        "imu/data_raw"));
    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}
 
extern "C" void app_main(void)
{   
    IC_SPI ic(&IC_spi_config);
    ic.begin();
    ic.readSTAT();
    ic.write_CFG1();
    ic.write_CFG2();
    ic.write_CFG3();
    while(1) {
        ic.readMVAL();
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // ic.write_CFG1();
    // ic.write_CFG2();
    // ic.write_CFG3();
 
    // #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    // rmw_uros_set_custom_transport(
    //     true,
    //     (void *) &uart_port,
    //     esp32_serial_open,
    //     esp32_serial_close,
    //     esp32_serial_write,
    //     esp32_serial_read
    // );
    // #else`
    // #error micro-ROS transports misconfigured
    // #endif  // RMW_UXRCE_TRANSPORT_CUSTOM
    // node_init();
    // spi.begin();
    // rosidl_runtime_c__String frame_id;
    // frame_id.data = "imu_link";
    // frame_id.size = strlen(frame_id.data);
    // frame_id.capacity = strlen(frame_id.data) + 1;
    // imu_msg.header.frame_id = frame_id;
    // while(1) {
    //     RCSOFTCHECK(rmw_uros_sync_session(1000));
    //     imu_msg.header.stamp.sec = rmw_uros_epoch_millis();
    //     imu_msg.linear_acceleration.x = spi.get_accel_x();
    //     imu_msg.linear_acceleration.y = spi.get_accel_y();
    //     imu_msg.linear_acceleration.z = spi.get_accel_z();
    //     imu_msg.angular_velocity.x = spi.get_gyro_x();
    //     imu_msg.angular_velocity.y = spi.get_gyro_y();
    //     imu_msg.angular_velocity.z = spi.get_gyro_z();
    //     publish_imuData();
    // }
}   
