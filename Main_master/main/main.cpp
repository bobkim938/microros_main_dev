#include <iostream>
#include "IMU_SPI.h"
#include "i2c_master.h"
#include "IC_spi.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <sensor_msgs/msg/imu.h>
#include <std_msgs/msg/int32_multi_array.h>
#include <std_msgs/msg/multi_array_dimension.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

using namespace std;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

rcl_publisher_t publisher;
rcl_publisher_t pub_encoder;
sensor_msgs__msg__Imu imu_msg = {};
std_msgs__msg__Int32MultiArray encoder_msg;

static size_t uart_port = UART_NUM_0; // UART port for Micro_ROS

DK42688_SPI_Config IMU_spi_config = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_10,
    .init_bus = 0 
};

IC_SPI_Config IC_left = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_9,
    .hwa = 0b0000,
    .init_bus = 1
};

IC_SPI_Config IC_right = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_35,
    .hwa = 0b0000,
    .init_bus = 1 
};

i2c_master_config i2c_config = {
    .sda = GPIO_NUM_18, 
    .scl = GPIO_NUM_19, 
};


void publish_imuData() {
    rcl_ret_t rc;
    rc = rcl_publish(&publisher, &imu_msg, NULL);
    if (rc != RCL_RET_OK) {
        printf("Failed to publish IMU data");
    }
}

void publish_encoderData() {
    rcl_ret_t rc;
    rc = rcl_publish(&pub_encoder, &encoder_msg, NULL);
    if (rc != RCL_RET_OK) {
        printf("Failed to publish encoder data");
    }
}
 
void node_init() {
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support; 
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
 
    // create node
    rcl_node_t node;
    RCCHECK(rclc_node_init_default(&node, "ESP32", "", &support));
 
    // create publisher for IMU data
    RCCHECK(rclc_publisher_init_best_effort(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));

    // create publisher for encoder data
    encoder_msg.data.capacity = 2;
    encoder_msg.data.size = 2; // Set size to 2 since we will populate 2 elements
    encoder_msg.data.data = (int32_t *)malloc(encoder_msg.data.capacity * sizeof(int32_t));
    if (encoder_msg.data.data == NULL) {
        printf("Failed to allocate memory for encoder_msg.data.data\n");
        vTaskDelete(NULL);
    }
    RCCHECK(rclc_publisher_init_best_effort(
        &pub_encoder,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
        "encoder_data"));

    rclc_executor_t executor;
    RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
}

extern "C" void app_main(void)
{   
    // #if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
    // rmw_uros_set_custom_transport(
    //     true,
    //     (void *) &uart_port,
    //     esp32_serial_open,
    //     esp32_serial_close,
    //     esp32_serial_write,
    //     esp32_serial_read
    // );
    // #else
    // #error micro-ROS transports misconfigured
    // #endif  // RMW_UXRCE_TRANSPORT_CUSTOM

    // node_init();
    DK42688_SPI IMU(&IMU_spi_config);
    // IC_SPI ic_left(&IC_left);
    // IC_SPI ic_right(&IC_right);
    IMU.begin();    
    // ic_left.begin();
    // ic_right.begin();
    // rosidl_runtime_c__String frame_id;
    // frame_id.data = "imu_link";
    // frame_id.size = strlen(frame_id.data);
    // frame_id.capacity = strlen(frame_id.data) + 1;
    // imu_msg.header.frame_id = frame_id;
    // ic_left.readSTAT();
    // ic_right.readSTAT();

    // while (1) {
    //     // encoder_msg.data.data[0] = ic_left.readMVAL();
    //     // encoder_msg.data.data[1] = ic_right.readMVAL();
    //     // encoder_msg.data.size = 2; // Ensure size is set correctly each time

    //     RCSOFTCHECK(rmw_uros_sync_session(1000));
    //     imu_msg.header.stamp.sec = rmw_uros_epoch_millis()/1000.0;
    //     imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();

    //     imu_msg.linear_acceleration.x = IMU.get_accel_x();
    //     imu_msg.linear_acceleration.y = IMU.get_accel_y();
    //     imu_msg.linear_acceleration.z = IMU.get_accel_z();
    //     imu_msg.angular_velocity.x = IMU.get_gyro_x();
    //     imu_msg.angular_velocity.y = IMU.get_gyro_y();
    //     imu_msg.angular_velocity.z = IMU.get_gyro_z();
    //     publish_imuData();
    // }
}   