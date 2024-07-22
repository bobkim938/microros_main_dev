#include <iostream>
#include "IMU_SPI.h"
#include "i2c_master.h"
#include "IC_spi.h"
#include "E-STOP.h"

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
#include <std_msgs/msg/u_int32.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uros_network_interfaces.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include "esp32_serial_transport.h"

using namespace std;

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

#define DO10 GPIO_NUM_36
#define DO11 GPIO_NUM_37
#define DO12 GPIO_NUM_38
#define DO13 GPIO_NUM_39
#define DO14 GPIO_NUM_40
#define DO15 GPIO_NUM_41
#define DO16 GPIO_NUM_42
#define DO17 GPIO_NUM_14
#define DO18 GPIO_NUM_21
#define DO19 GPIO_NUM_47

rcl_publisher_t publisher;
rcl_publisher_t pub_encoder;
rcl_publisher_t di_publisher;
rcl_subscription_t DO_subscriber;
rclc_executor_t executor;
sensor_msgs__msg__Imu imu_msg = {};
std_msgs__msg__Int32MultiArray encoder_msg;
std_msgs__msg__UInt32 di_data;
std_msgs__msg__UInt32 recv_msg;

bool new_DO = false;
bool new_DI = false;

uint8_t slave_do[3] = {0xBB, 0x00, 0x00}; // first byte to indicating DO cmd, second bit 2 MSB
uint16_t master_do = 0;

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
    .sda = GPIO_NUM_15, 
    .scl = GPIO_NUM_16, 
    .slaveAddr = 0x0A
};

void reset_gpio() {
    gpio_reset_pin(DO10); // 1024
    gpio_reset_pin(DO11); // 2048
    gpio_reset_pin(DO12); // 4096
    gpio_reset_pin(DO13); // 8192
    gpio_reset_pin(DO14); // 16384
    gpio_reset_pin(DO15); // 32768
    gpio_reset_pin(DO16); // 65536
    gpio_reset_pin(DO17); // 131072
    gpio_reset_pin(DO18); // 262144
    gpio_reset_pin(DO19); // 524288

    gpio_set_direction(DO10, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO11, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO12, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO13, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO14, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO15, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO16, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO17, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO18, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO19, GPIO_MODE_OUTPUT);
}

void set_DO() {
    bool do10 = master_do & 0x0001;
    bool do11 = master_do & 0x0002;
    bool do12 = master_do & 0x0004;
    bool do13 = master_do & 0x0008;
    bool do14 = master_do & 0x0010;
    bool do15 = master_do & 0x0020;
    bool do16 = master_do & 0x0040;
    bool do17 = master_do & 0x0080;
    bool do18 = master_do & 0x0100;
    bool do19 = master_do & 0x0200;

    gpio_set_level(DO10, do10);
    gpio_set_level(DO11, do11);
    gpio_set_level(DO12, do12);
    gpio_set_level(DO13, do13);
    gpio_set_level(DO14, do14);
    gpio_set_level(DO15, do15);
    gpio_set_level(DO16, do16);
    gpio_set_level(DO17, do17);
    gpio_set_level(DO18, do18);
    gpio_set_level(DO19, do19);
}


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

void publish_DI() {
    rcl_ret_t rc;
    rc = rcl_publish(&di_publisher, &di_data, NULL);
    if (rc != RCL_RET_OK) {
        printf("Failed to publish DI data");
    }
}

void doSub_callback(const void* msgin) {
    const std_msgs__msg__UInt32 * msg = (const std_msgs__msg__UInt32 *)msgin;
    uint32_t do_data = msg->data;
    slave_do[2] = do_data & 0x000000FF; // Do0-DO7 (Slave)
    slave_do[1] = (do_data & 0x00000300) >> 8; // Do8-DO9 (Slave)
    master_do = (do_data & 0x000FFC00) >> 10; // Do10-DO19 (Master)
    new_DO = true;
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

    // // create publisher for encoder data
    // encoder_msg.data.capacity = 2;
    // encoder_msg.data.size = 2;
    // encoder_msg.data.data = (int32_t *)malloc(encoder_msg.data.capacity * sizeof(int32_t));
    // if (encoder_msg.data.data == NULL) {
    //     printf("Failed to allocate memory for encoder_msg.data.data\n");
    //     vTaskDelete(NULL);
    // }
    // RCCHECK(rclc_publisher_init_best_effort(
    //     &pub_encoder,
    //     &node,
    //     ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    //     "encoder_data"));

    // create publisher for DI data
    RCCHECK(rclc_publisher_init_default(
        &di_publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, UInt32),
        "DI_data"));

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
    uint8_t slave_do[3] = {0xBB, 0x01, 0xFF}; // first byte to indicating DO cmd, second bit 2 MSB
    i2c_master i2c(&i2c_config);
    i2c.begin();
    ESTOP estop;
    estop.begin();

    while(1) {
        // i2c.i2c_send_DO(slave_do);
        di_data.data = i2c.read_di();
        // i2c.check_BATSW();
        // i2c.cntrl_BMSpass(0x01);
        // publish_DI();
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    // DK42688_SPI IMU(&IMU_spi_config);
    // IC_SPI ic_left(&IC_left);
    // IC_SPI ic_right(&IC_right);
    // IMU.begin();    
    // ic_left.begin();
    // ic_right.begin();
    // rosidl_runtime_c__String frame_id;
    // frame_id.data = "imu_link";
    // frame_id.size = strlen(frame_id.data);
    // frame_id.capacity = strlen(frame_id.data) + 1;
    // imu_msg.header.frame_id = frame_id;
    // // di_data.data = ic_left.readSTAT();
    // di_data.data = ic_right.readSTAT();

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
    //     publish_DI();
    //     publish_imuData();
    // }
}   