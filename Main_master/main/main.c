#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/string_utilities.h>

//#include <std_msgs/msg/int32.h>
//#include <std_msgs/msg/int32_multi_array.h>
//#include <std_msgs/msg/multi_array_dimension.h>

#include <geometry_msgs/msg/twist.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/battery_state.h>
#include <nav_msgs/msg/odometry.h>
#include <shoalbot_msgs/msg/bms_state.h>
#include <shoalbot_msgs/msg/di_state.h>
#include <shoalbot_msgs/msg/do_cmd.h>
#include <shoalbot_msgs/msg/encoder_count.h>
#include <shoalbot_msgs/msg/kinco_mode.h>
#include <shoalbot_msgs/msg/led_cmd.h>
#include <shoalbot_msgs/msg/position_cmd.h>
#include <shoalbot_msgs/msg/speed_cmd.h>

#include "esp32_serial_transport.h"
#include "kinco_can.h"
#include "amip4k_spi.h"
#include "icm42688_spi.h"
#include "movingaverage_filter.h"
#include "bms_485.h"
#include "shoalbot_master_i2c.h"
#include "estop.h"

#include "esp_task_wdt.h"

//#include "esp_intr_alloc.h"
//#include "esp_intr_types.h"

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc);vTaskDelete(NULL);}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printf("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

//#define ROS_NAMESPACE CONFIG_MICRO_ROS_NAMESPACE

#define JETSON_22 GPIO_NUM_0
#define PWM_1 GPIO_NUM_1
#define PWM_2 GPIO_NUM_2
#define RS2_DE GPIO_NUM_3
#define DO_6 GPIO_NUM_4
#define DO_7 GPIO_NUM_5
#define CAN1_TX GPIO_NUM_6
#define CAN1_RX GPIO_NUM_7
#define RS1_DE GPIO_NUM_8
#define DO_17 GPIO_NUM_14
#define RS1_TX GPIO_NUM_17
#define RS1_RX GPIO_NUM_18
#define RS2_TX GPIO_NUM_19
#define RS2_RX GPIO_NUM_20
#define DO_18 GPIO_NUM_21
#define DO_10 GPIO_NUM_36
#define DO_11 GPIO_NUM_37
#define DO_12 GPIO_NUM_38
#define DO_13 GPIO_NUM_39
#define DO_14 GPIO_NUM_40
#define DO_15 GPIO_NUM_41
#define DO_16 GPIO_NUM_42
#define DO_19 GPIO_NUM_47
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_16

unsigned long long time_offset = 0;
unsigned long prev_odom_update = 0;
unsigned long prev_time_update = 0;
float x_pos_ = 0.0;
float y_pos_ = 0.0;
float heading_ = 0.0;
const int interpolation_rate_ = 4;
const int encoder_resolution_ = 1024;
const float wheel_diameter_ = 0.15; 
const float wheel_base_ = 0.469; 
const int gear_ratio_ = 15;
int cpr = interpolation_rate_ * encoder_resolution_;
double gyro_x, gyro_y, gyro_z, accel_x, accel_y, accel_z; 

int32_t left_speed, right_speed;
int16_t left_count_now, left_count_prev, right_count_now, right_count_prev;
int32_t left_counter, left_counter_prev, right_counter, right_counter_prev;
float left_speed_m, right_speed_m;

int32_t	left_kinco_pos, right_kinco_pos, left_kinco_speed, right_kinco_speed; //TO remove after acceptance test
uint32_t profile_speed = 900, profile_acc = 15, profile_dec = 15; //TO remove after acceptance test

rcl_publisher_t imu_publisher/*, odom_publisher, bms_publisher, di_publisher*/;
rcl_publisher_t bms_publisher, di_publisher, encoder_publisher; 
sensor_msgs__msg__Imu imu_msg;
//sensor_msgs__msg__BatteryState bms_msg;
//nav_msgs__msg__Odometry odom_msg;
//std_msgs__msg__Int32 di_msg;
shoalbot_msgs__msg__BmsState bms_msg;
shoalbot_msgs__msg__DiState di_msg;
shoalbot_msgs__msg__EncoderCount encoder_msg;

//rcl_subscription_t twist_subscriber, do_subscriber;
rcl_subscription_t do_subscriber, kinco_subscriber, led_subscriber, position_subscriber, speed_subscriber;
//geometry_msgs__msg__Twist twist_msg;
//std_msgs__msg__Int32 do_msg;
shoalbot_msgs__msg__DoCmd do_msg;
shoalbot_msgs__msg__KincoMode kinco_msg;
shoalbot_msgs__msg__LedCmd led_msg;
shoalbot_msgs__msg__PositionCmd position_msg;
shoalbot_msgs__msg__SpeedCmd speed_msg;

static size_t uart_port = UART_NUM_0; // UART port for micro-ROS

// int32_t left_input_filtered, right_input_filtered;
// float left_speed_filtered, right_speed_filtered;

int32_t di_buffer;
bool new_DO = false;
bool new_DI = false;
uint8_t slave_do[5] = {0xBB, 0x0B, 0x00, 0x00, 0x00}; // first byte to indicating DO cmd, second bit 2 MSB
uint16_t master_do = 0;
uint32_t bat_percentage = 0;

i2c_master_config shoalbot_i2c_config = {
	.sda = I2C_SDA,
	.scl = I2C_SCL,
	.slaveAddr = 0x0A
};

shoalbot_master_i2c shoalbot_i2c;
shoalbot_bms bms;
sboalbot_amip4k amip4k;
shoalbot_icm42688 icm42688;
shoalbot_estop estop;
MovingAverageFilter left_speed_filter, right_speed_filter;

void reset_gpio() {
	gpio_reset_pin(DO_6);
	gpio_reset_pin(DO_7);
    gpio_reset_pin(DO_10); // 1024
    gpio_reset_pin(DO_11); // 2048
    gpio_reset_pin(DO_12); // 4096
    gpio_reset_pin(DO_13); // 8192
    gpio_reset_pin(DO_14); // 16384
    gpio_reset_pin(DO_15); // 32768
    gpio_reset_pin(DO_16); // 65536
    gpio_reset_pin(DO_17); // 131072
    gpio_reset_pin(DO_18); // 262144
    gpio_reset_pin(DO_19); // 524288

	gpio_set_direction(DO_6, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_7, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_10, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_11, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_12, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_13, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_14, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_15, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_16, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_17, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_18, GPIO_MODE_OUTPUT);
    gpio_set_direction(DO_19, GPIO_MODE_OUTPUT);
}

void set_DO(void* arg) {
	// uint16_t master_do = 0;
	while(1) {
		if(new_DO) {
			new_DO = false;
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
			bool do6 = master_do & 0x0400;
			bool do7 = master_do & 0x0800;

			gpio_set_level(DO_10, do10);
			gpio_set_level(DO_11, do11);
			gpio_set_level(DO_12, do12);
			gpio_set_level(DO_13, do13);
			gpio_set_level(DO_14, do14);
			gpio_set_level(DO_15, do15);
			gpio_set_level(DO_16, do16);
			gpio_set_level(DO_17, do17);
			gpio_set_level(DO_18, do18);
			gpio_set_level(DO_19, do19);
			gpio_set_level(DO_6, do6);
			gpio_set_level(DO_7, do7);
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}

/*
void twist_callback(const void * msgin) {
	left_speed = (twist_msg.linear.x - (twist_msg.angular.z*wheel_base_*0.5)) / (wheel_diameter_*M_PI) * 60 * gear_ratio_; 
	right_speed = (twist_msg.linear.x + (twist_msg.angular.z*wheel_base_*0.5)) / (wheel_diameter_*M_PI) * 60 * gear_ratio_; 
	left_input_filtered = left_input_filter.process(left_speed);
	right_input_filtered = right_input_filter.process(right_speed);
}

void do_callback(const void * msgin) {
	uint32_t do_data = do_msg.data;
	slave_do[2] = do_data & 0x000000FF; // Do0-DO7 (Slave)
    slave_do[1] = (do_data & 0x00000300) >> 8; // Do8-DO9 (Slave)
    master_do = (do_data & 0x000FFC00) >> 10; // Do10-DO19 (Master)
    new_DO = true;
}
*/

void do_callback(const void * msgin) {
	uint32_t do_data = do_msg.data;
	// slave_do[2] = do_data & 0x000000FF; // Do0-DO7 (Slave)
    // slave_do[1] = (do_data & 0x00000300) >> 8; // Do8-DO9 (Slave)
	slave_do[1] = ((do_data >> 2 & 0xC0)) | (do_data & 0x3F); // DO9-DO8 (Slave), DO5-DO0 (Slave)
    master_do = (do_data & 0x000FFC00) >> 10; // Do10-DO19 (Master)
	master_do |= (((do_data >> 6) & 0x03) << 10); // DO6-DO7 (Master)
    new_DO = true;
}
int8_t kinco_mode = 3;
void kinco_callback(const void * msgin) {
	kinco_mode = kinco_msg.mode;
	setModesOfOperation(1, kinco_mode);
	setModesOfOperation(2, kinco_mode);
	if (kinco_mode==1) {
		setDin2Function(1, 0x4000); //set din 2 to activate command
		setDin2Function(2, 0x4000);
	}
}

void led_callback(const void * msgin) {
	uint8_t color_index = led_msg.color;
	slave_do[3] = color_index; // LSB PART OF NAV STATUS
}

void position_callback(const void * msgin) {
	left_kinco_pos = position_msg.left;
	right_kinco_pos = position_msg.right;
	profile_speed = position_msg.profile_speed;
	profile_acc = position_msg.profile_acc;
	profile_dec = position_msg.profile_dec;
	setTargetPosition(1, left_kinco_pos);
	setTargetPosition(2, right_kinco_pos);

	vTaskDelay(pdMS_TO_TICKS(2));
	setProfileSpeed(1, profile_speed); //set rpm for position control mode
	setProfileSpeed(2, profile_speed);
	vTaskDelay(pdMS_TO_TICKS(2));
	setProfileAcceleration(1, profile_acc); //set rps/s for position control mode
	setProfileAcceleration(2, profile_acc);
	vTaskDelay(pdMS_TO_TICKS(2));
	setProfileDeceleration(1, profile_dec); 
	setProfileDeceleration(2, profile_dec);
	vTaskDelay(pdMS_TO_TICKS(2));

	setDinSimulate(1, 0x0002);
	setDinSimulate(2, 0x0002);
	vTaskDelay(pdMS_TO_TICKS(2));	
	setDinSimulate(1, 0x0000);
	setDinSimulate(2, 0x0000);
}

void speed_callback(const void * msgin) {
	left_kinco_speed = speed_msg.left;
	right_kinco_speed = speed_msg.right;
}
/*
void battery_ros_init(void) { // Initializes the ROS topic information for Batery
	bms_msg.power_supply_technology = 4;
	bms_msg.present = 1;
//	bms_msg.location = micro_ros_string_utilities_set(bms_msg.location, "amr_1");
//	bms_msg.serial_number = micro_ros_string_utilities_set(bms_msg.serial_number, "");
//	bms_msg.header.frame_id = micro_ros_string_utilities_set(bms_msg.header.frame_id, "battery");
}
*/

void imu_ros_init(void) { // Initializes the ROS topic information for IMU
	imu_msg.angular_velocity.x = 0;
	imu_msg.angular_velocity.y = 0;
	imu_msg.angular_velocity.z = 0;
	imu_msg.linear_acceleration.x = 0;
	imu_msg.linear_acceleration.y = 0;
	imu_msg.linear_acceleration.z = 0;
	imu_msg.orientation.x = 0;
	imu_msg.orientation.y = 0;
	imu_msg.orientation.z = 0;
	imu_msg.orientation.w = 1;
	imu_msg.angular_velocity_covariance[0] = 0.0002;
	imu_msg.angular_velocity_covariance[4] = 0.0002;
	imu_msg.angular_velocity_covariance[8] = 0.0002;
	imu_msg.linear_acceleration_covariance[0] = 0.0012;
	imu_msg.linear_acceleration_covariance[4] = 0.0012;
	imu_msg.linear_acceleration_covariance[8] = 0.0012;
	imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "imu_link");
	imu_msg.orientation_covariance[0] = 0.00001;
	imu_msg.orientation_covariance[4] = 0.00001;
	imu_msg.orientation_covariance[8] = 0.00001;
}

/*
void odom_ros_init(void) {
	odom_msg.header.frame_id = micro_ros_string_utilities_set(odom_msg.header.frame_id, "odom");
	odom_msg.child_frame_id = micro_ros_string_utilities_set(odom_msg.child_frame_id, "base_link");
	odom_msg.pose.pose.position.z = 0.0;
	odom_msg.pose.covariance[0] = 0.0001;
	odom_msg.pose.covariance[7] = 0.0001;
	odom_msg.pose.covariance[35] = 0.0001;
	odom_msg.twist.twist.linear.y = 0.0;
	odom_msg.twist.twist.linear.z = 0.0;
	odom_msg.twist.twist.angular.x = 0.0;
	odom_msg.twist.twist.angular.y = 0.0;
	odom_msg.twist.covariance[0] = 0.0001;
	odom_msg.twist.covariance[7] = 0.0001;
	odom_msg.twist.covariance[35] = 0.0001;
}
*/
/*
void odom_euler_to_quat(float roll, float pitch, float yaw, float *q) {
	float cy = cos(yaw * 0.5);
	float sy = sin(yaw * 0.5);
	float cp = cos(pitch * 0.5);
	float sp = sin(pitch * 0.5);
	float cr = cos(roll * 0.5);
	float sr = sin(roll * 0.5);
	q[0] = cy * cp * cr + sy * sp * sr;
	q[1] = cy * cp * sr - sy * sp * cr;
	q[2] = sy * cp * sr + cy * sp * cr;
	q[3] = sy * cp * cr - cy * sp * sr;
}
*/

unsigned long get_millisecond(void) { // Get the number of seconds since boot
	return (unsigned long) (esp_timer_get_time() / 1000ULL);
}

static void sync_time(void) { // Calculate the time difference between the microROS agent and the MCU
	unsigned long now = get_millisecond();
	RCSOFTCHECK(rmw_uros_sync_session(10));
	unsigned long long ros_time_ms = rmw_uros_epoch_millis();
	time_offset = ros_time_ms - now;    
}

struct timespec get_timespec(void) { // Get timestamp
	struct timespec tp = {};
	unsigned long long now = get_millisecond() + time_offset; // deviation of synchronous time
	tp.tv_sec = now / 1000;
	tp.tv_nsec = (now % 1000) * 1000000;
	return tp;
}

/*
void odom_update(float vel_dt, float linear_vel_x, float linear_vel_y, float angular_vel_z) {
	float delta_heading = angular_vel_z * vel_dt; // radians
	float cos_h = cos(heading_);
	float sin_h = sin(heading_);
	float delta_x = (linear_vel_x * cos_h - linear_vel_y * sin_h) * vel_dt; // m
	float delta_y = (linear_vel_x * sin_h + linear_vel_y * cos_h) * vel_dt; // m
	// calculate current position of the robot
	x_pos_ += delta_x;
	y_pos_ += delta_y;
	heading_ += delta_heading;
	// calculate robot's heading in quaternion angle. ROS has a function to calculate yaw in quaternion angle
	float q[4];
	odom_euler_to_quat(0, 0, heading_, q);
	// robot's position in x,y, and z
	odom_msg.pose.pose.position.x = x_pos_;
	odom_msg.pose.pose.position.y = y_pos_;
	// robot's heading in quaternion
	odom_msg.pose.pose.orientation.x = (double)q[1];
	odom_msg.pose.pose.orientation.y = (double)q[2];
	odom_msg.pose.pose.orientation.z = (double)q[3];
	odom_msg.pose.pose.orientation.w = (double)q[0];
	// linear speed from encoders
	odom_msg.twist.twist.linear.x = linear_vel_x;
	odom_msg.twist.twist.linear.y = linear_vel_y;
	// angular speed from encoders
	odom_msg.twist.twist.angular.z = angular_vel_z;
}
*/

void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		struct timespec time_stamp = get_timespec();
		imu_msg.header.stamp.sec = time_stamp.tv_sec;
		imu_msg.header.stamp.nanosec = time_stamp.tv_nsec;
		//imu_msg.header.stamp.sec = rmw_uros_epoch_millis()/1000.0;
		//imu_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
		imu_msg.angular_velocity.x = gyro_x; 
		imu_msg.angular_velocity.y = gyro_y; 
		imu_msg.angular_velocity.z = gyro_z; 
		imu_msg.linear_acceleration.x = accel_x; 
		imu_msg.linear_acceleration.y = accel_y; 
		imu_msg.linear_acceleration.z = accel_z; 
		RCSOFTCHECK(rcl_publish(&imu_publisher, &imu_msg, NULL));
	}
}

// void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
// 	RCLC_UNUSED(last_call_time);
// 	if (timer != NULL) {
// /*		unsigned long now = get_millisecond();
// 		float vel_dt = (now - prev_odom_update) / 1000.0;
// 		prev_odom_update = now;
// 		left_speed_m = (left_counter - left_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
// 		right_speed_m = (right_counter - right_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
// 		left_speed_filtered = MovingAverageFilter_process(&left_speed_filter ,left_speed_m);
// 		right_speed_filtered = MovingAverageFilter_process(&right_speed_filter, right_speed_m);
// 		float Vx = (right_speed_filtered + left_speed_filtered) * 0.5; //robot m/s
// 		float Vy = 0;
// 		float Wz = (right_speed_filtered - left_speed_filtered) / wheel_base_; // robot rad/s
// 		odom_update(vel_dt, Vx, Vy, Wz);
// //		test_msg.data = right_speed_m;
// 		test_msg.data = right_speed_filtered;
// 		left_counter_prev = left_counter;
// 		right_counter_prev = right_counter; */
// 		struct timespec time_stamp = get_timespec();
// 		odom_msg.header.stamp.sec = time_stamp.tv_sec;
// 		odom_msg.header.stamp.nanosec = time_stamp.tv_nsec;
// 		//odom_msg.header.stamp.sec = rmw_uros_epoch_millis()/1000.0;
// 		//odom_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
// 		RCSOFTCHECK(rcl_publish(&odom_publisher, &odom_msg, NULL));
// 	}
// }
/*
void bms_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		struct timespec time_stamp = get_timespec();
		bms_msg.header.stamp.sec = time_stamp.tv_sec;
		bms_msg.header.stamp.nanosec = time_stamp.tv_nsec;
		//bms_msg.header.stamp.sec = rmw_uros_epoch_millis()/1000.0;
		//bms_msg.header.stamp.nanosec = rmw_uros_epoch_nanos();
		RCSOFTCHECK(rcl_publish(&bms_publisher, &bms_msg, NULL));
	}
}

void di_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		di_msg.data = di_buffer; 
		RCSOFTCHECK(rcl_publish(&di_publisher, &di_msg, NULL));
	}
}
*/
void bms_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&bms_publisher, &bms_msg, NULL));
	}
}

void di_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		di_msg.data = di_buffer;
		RCSOFTCHECK(rcl_publish(&di_publisher, &di_msg, NULL));
	}
}

void encoder_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
	RCLC_UNUSED(last_call_time);
	if (timer != NULL) {
		RCSOFTCHECK(rcl_publish(&encoder_publisher, &encoder_msg, NULL));
	}
}

void micro_ros_task(void * arg) {
	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	// Create init_options
	rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
	//rmw_init_options_t *rmw_options = rcl_init_options_get_rmw_init_options(&init_options);
	RCCHECK(rcl_init_options_init(&init_options, allocator));
	//RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));
	
	// Setup support structure
	RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));
	//RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
	
	// Create node
	rcl_node_t node = rcl_get_zero_initialized_node();
	//rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "shoalbot_master", "", &support));

	// create publisher
	//RCCHECK(rclc_publisher_init_best_effort(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
	RCCHECK(rclc_publisher_init_default(&imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data_raw"));
//	RCCHECK(rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom/data_raw"));
//	RCCHECK(rclc_publisher_init_default(&bms_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState), "bms"));
//	RCCHECK(rclc_publisher_init_default(&di_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "di"));
	RCCHECK(rclc_publisher_init_default(&bms_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, BmsState), "bms"));
	RCCHECK(rclc_publisher_init_default(&di_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, DiState), "di"));
	RCCHECK(rclc_publisher_init_default(&encoder_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, EncoderCount), "encoder"));

	// Create subscriber
//	RCCHECK(rclc_subscription_init_default(&twist_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel"))
//	RCCHECK(rclc_subscription_init_default(&do_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "do"))
	RCCHECK(rclc_subscription_init_default(&do_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, DoCmd), "do"));
	RCCHECK(rclc_subscription_init_default(&kinco_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, KincoMode), "kinco"));
	RCCHECK(rclc_subscription_init_default(&led_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, LedCmd), "led"));
	RCCHECK(rclc_subscription_init_default(&position_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, PositionCmd), "position"));
	RCCHECK(rclc_subscription_init_default(&speed_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(shoalbot_msgs, msg, SpeedCmd), "speed"));

	// Create timer
	rcl_timer_t imu_timer = rcl_get_zero_initialized_timer(); 
//	rcl_timer_t odom_timer = rcl_get_zero_initialized_timer();
//	rcl_timer_t bms_timer = rcl_get_zero_initialized_timer();
//	rcl_timer_t di_timer = rcl_get_zero_initialized_timer();
	rcl_timer_t bms_timer = rcl_get_zero_initialized_timer();
	rcl_timer_t di_timer = rcl_get_zero_initialized_timer();
	rcl_timer_t encoder_timer = rcl_get_zero_initialized_timer();
	RCCHECK(rclc_timer_init_default(&imu_timer, &support, RCL_MS_TO_NS(20), imu_timer_callback));
//	RCCHECK(rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(50), odom_timer_callback));
//	RCCHECK(rclc_timer_init_default(&bms_timer, &support, RCL_S_TO_NS(5), bms_timer_callback));
//	RCCHECK(rclc_timer_init_default(&di_timer, &support, RCL_MS_TO_NS(200), di_timer_callback));
	RCCHECK(rclc_timer_init_default(&bms_timer, &support, RCL_S_TO_NS(5), bms_timer_callback));
	RCCHECK(rclc_timer_init_default(&di_timer, &support, RCL_MS_TO_NS(200), di_timer_callback));
	RCCHECK(rclc_timer_init_default(&encoder_timer, &support, RCL_MS_TO_NS(50), encoder_timer_callback));

	// Create executor
	//rclc_executor_t executor;
	rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
	RCCHECK(rclc_executor_init(&executor, &support.context, 9, &allocator));
	unsigned int rcl_wait_timeout = 1000; 
	RCCHECK(rclc_executor_set_timeout(&executor, RCL_MS_TO_NS(rcl_wait_timeout)));

	// Add timer and subscriber to executor
	RCCHECK(rclc_executor_add_timer(&executor, &imu_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &odom_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &bms_timer));
//	RCCHECK(rclc_executor_add_timer(&executor, &di_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &bms_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &di_timer));
	RCCHECK(rclc_executor_add_timer(&executor, &encoder_timer));
//	RCCHECK(rclc_executor_add_subscription(&executor, &twist_subscriber, &twist_msg, &twist_callback, ON_NEW_DATA));
//	RCCHECK(rclc_executor_add_subscription(&executor, &do_subscriber, &do_msg, &do_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &do_subscriber, &do_msg, &do_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &kinco_subscriber, &kinco_msg, &kinco_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &led_subscriber, &led_msg, &led_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &position_subscriber, &position_msg, &position_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_subscription(&executor, &speed_subscriber, &speed_msg, &speed_callback, ON_NEW_DATA));

	sync_time();

	// Spin forever
	while(1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1)); //100
		usleep(1000); // 100000
	}

	// Free resources
//	RCCHECK(rcl_subscription_fini(&twist_subscriber, &node));
//	RCCHECK(rcl_subscription_fini(&do_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&do_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&kinco_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&led_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&position_subscriber, &node));
	RCCHECK(rcl_subscription_fini(&speed_subscriber, &node));
	RCCHECK(rcl_publisher_fini(&imu_publisher, &node));
//	RCCHECK(rcl_publisher_fini(&odom_publisher, &node));
//	RCCHECK(rcl_publisher_fini(&bms_publisher, &node));
//	RCCHECK(rcl_publisher_fini(&di_publisher, &node));
	RCCHECK(rcl_publisher_fini(&bms_publisher, &node));
	RCCHECK(rcl_publisher_fini(&di_publisher, &node));
	RCCHECK(rcl_publisher_fini(&encoder_publisher, &node));
	RCCHECK(rcl_node_fini(&node));

	vTaskDelete(NULL);
}
/*
void twai_task(void *arg) { // Kinco motor task
	while (1) {
		setTargetVelocity(1, left_input_filtered); 
		setTargetVelocity(2, right_input_filtered); 
	}
	vTaskDelete(NULL);
}
*/
void twai_task(void *arg) { // Kinco motor task
	while (1) {
		switch (kinco_mode) {
			case 1: // position control mode
				break;
			case 3:  // velocity control mode
				setTargetVelocity(1, left_kinco_speed); 
				setTargetVelocity(2, right_kinco_speed);
				break;
			default:
				break;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	vTaskDelete(NULL);
}

/*
void spi_task(void *arg) { // IMU and safety encoder data task
	while (1) {
		gyro_x = imu.get_gyro_x(); 
		gyro_y = imu.get_gyro_y(); 
		gyro_z = imu.get_gyro_z(); 
		accel_x = imu.get_accel_x(); 
		accel_y = imu.get_accel_y(); 
		accel_z = imu.get_accel_z(); 
		vTaskDelay(pdMS_TO_TICKS(2));

		left_count_now = left_encoder.readMVAL() * -1;
		right_count_now = right_encoder.readMVAL();

//		unsigned long now = get_millisecond();
//		float vel_dt = (now - prev_odom_update) / 1000.0;
		unsigned long now = esp_timer_get_time();
		float vel_dt = (now - prev_odom_update) / 1000000.0;
		prev_odom_update = now;

			
		//moving forward: -3 -2 -1 0 1 2 3
		//reset forward: 2045 2046 2047 0 1 2 3
		//moving backward: 3 2 1 0 -1 -2 -3
		//reset backward: -2045 -2046 -2047 -2048 -1 -2 -3

		if (right_count_now > right_count_prev) { // moving forward or reset after moving backward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter = right_counter - (2048 - right_count_now + right_count_prev); //reset backward
			else right_counter = right_counter + (right_count_now - right_count_prev); //moving forward
		}
		else if (right_count_now < right_count_prev) { //moving backward or reset after moving forward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter = right_counter + (right_count_now - right_count_prev + 2048); //reset forward
			else right_counter = right_counter - (right_count_prev - right_count_now); //moving backward
		}
		else {} 
		right_count_prev = right_count_now;

		if (left_count_now > left_count_prev) { // moving forward or reset after moving backward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter = left_counter - (2048 - left_count_now + left_count_prev); //reset backward
			else left_counter = left_counter + (left_count_now - left_count_prev); //moving forward
		}
		else if (left_count_now < left_count_prev) { //moving backward or reset after moving forward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter = left_counter + (left_count_now - left_count_prev + 2048);  //reset forward
			else left_counter = left_counter - (left_count_prev - left_count_now); //moving backward
		}
		else {} 
		left_count_prev = left_count_now;

//		unsigned long now = get_millisecond();
//		float vel_dt = (now - prev_odom_update) / 1000.0;
//		prev_odom_update = now;
		left_speed_m = (left_counter - left_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
		right_speed_m = (right_counter - right_counter_prev) / vel_dt / cpr * M_PI * wheel_diameter_; // measure m/s
		if (left_counter==left_counter_prev) left_speed_m = 0;
		if (right_counter==right_counter_prev) right_speed_m = 0;
		left_counter_prev = left_counter;
		right_counter_prev = right_counter;
		left_speed_filtered = left_speed_filter.process(left_speed_m);
		right_speed_filtered = right_speed_filter.process(right_speed_m);
		float Vx = (right_speed_filtered + left_speed_filtered) / 2.0; //robot m/s
		float Vy = 0;
		float Wz = (right_speed_filtered - left_speed_filtered) / wheel_base_; // robot rad/s
		odom_update(vel_dt, Vx, Vy, Wz);
		vTaskDelay(pdMS_TO_TICKS(2));

	}
	vTaskDelete(NULL);
}
*/
void spi_task(void *arg) { // IMU and safety encoder data task
	while (1) {
		gyro_x = icm42688_spi_get_gyro_x(&icm42688, 0); 
		gyro_y = icm42688_spi_get_gyro_y(&icm42688, 0); 
		gyro_z = icm42688_spi_get_gyro_z(&icm42688, 0); 
		accel_x = icm42688_spi_get_accel_x(&icm42688,0); 
		accel_y = icm42688_spi_get_accel_y(&icm42688, 0); 
		accel_z = icm42688_spi_get_accel_z(&icm42688, 0); 
		vTaskDelay(pdMS_TO_TICKS(2));

		left_count_now = amip4k_spi_readMVAL(&amip4k, 'L'); //beware of the direction
		right_count_now = amip4k_spi_readMVAL(&amip4k, 'R') * -1; //beware of the direction
		unsigned long now = esp_timer_get_time();
		prev_odom_update = now;
		/*	
		moving forward: -3 -2 -1 0 1 2 3
		reset forward: 2045 2046 2047 0 1 2 3
		moving backward: 3 2 1 0 -1 -2 -3
		reset backward: -2045 -2046 -2047 -2048 -1 -2 -3
		*/
		if (right_count_now > right_count_prev) { // moving forward or reset after moving backward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter = right_counter - (2048 - right_count_now + right_count_prev); //reset backward
			else right_counter = right_counter + (right_count_now - right_count_prev); //moving forward
		}
		else if (right_count_now < right_count_prev) { //moving backward or reset after moving forward
			if (abs(right_count_now - right_count_prev) > 1024) right_counter = right_counter + (right_count_now - right_count_prev + 2048); //reset forward
			else right_counter = right_counter - (right_count_prev - right_count_now); //moving backward
		}
		else {} 
		right_count_prev = right_count_now;

		if (left_count_now > left_count_prev) { // moving forward or reset after moving backward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter = left_counter - (2048 - left_count_now + left_count_prev); //reset backward
			else left_counter = left_counter + (left_count_now - left_count_prev); //moving forward
		}
		else if (left_count_now < left_count_prev) { //moving backward or reset after moving forward
			if (abs(left_count_now - left_count_prev) > 1024) left_counter = left_counter + (left_count_now - left_count_prev + 2048);  //reset forward
			else left_counter = left_counter - (left_count_prev - left_count_now); //moving backward
		}
		else {} 
		left_count_prev = left_count_now;
		encoder_msg.left = left_counter;
		encoder_msg.right = right_counter;
		vTaskDelay(pdMS_TO_TICKS(2));
	}
	vTaskDelete(NULL);
}

/*
void rs485_task(void *arg) { // BMS task
	while (1) {
        bms.getBMSData();
        bms_msg.voltage = bms.getVoltage();
        bms_msg.temperature = bms.getTemperature();
        bms_msg.current = bms.getCurrent();
        bms_msg.charge = bms.getCharge();
        bms_msg.capacity = bms.getCapacity();
        bms_msg.design_capacity = bms.getCapacity();
        bms_msg.percentage = bms.getCharge() / bms.getCapacity();
        bms_msg.cell_temperature.size = 3;
        bms_msg.cell_temperature.data = bms.getCellTemperature();
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}
*/
void rs485_task(void *arg) { // BMS task
	while (1) {
		bms_485_getBMSData(&bms);
        bms_msg.voltage = bms_485_getVoltage(&bms);
        bms_msg.temperature = bms_485_getTemperature(&bms);
        bms_msg.current = bms_485_getCurrent(&bms);
        bms_msg.battery_level = bms_485_getCharge(&bms) / bms_485_getCapacity(&bms) * 100;
		bms_msg.charge_cycle = bms_485_getCycle(&bms);
		vTaskDelay(pdMS_TO_TICKS(2000));
		// printf("BMS task\n");
	}
	vTaskDelete(NULL);
}

void i2c_task(void *arg) { // I2C master task
	//i2c.cntrl_BMSpass(0b001); // Pass2 0, Pass1 0, BMS 1
	// DO 9 8 7 6 5 4 3 2 1 0
	//    0 0 0 0 0 0 1 0 1 1
	// slave_do[0] = {0xBB}
	// slave_do[1] = {DO9, DO8, DO5, DO4, DO3, DO2, DO1, DO0} LSB PART OF SLAVE DO
	// slave_do[2] = {0, 0, 0, 0, 0, 0, 0, nav_st9, nav_st8} MSB PART OF NAV STATUS
	// slave_do[3] = {nav_st7, nav_st6, nav_st5, nav_st4, nav_st3, nav_st2, nav_st1, nav_st0} LSB PART OF NAV STATUS
	// slave_do[4] = BMS PERCENTAGE

	float a = 97.3;
	slave_do[4] = (uint8_t) a;
	//slave_do[4] = 1;
	while (1) {
		di_buffer =shoalbot_master_i2c_read_state(&shoalbot_i2c);
		vTaskDelay(pdMS_TO_TICKS(100));
		shoalbot_master_i2c_i2c_send_DO(&shoalbot_i2c, slave_do);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	vTaskDelete(NULL);
}

/******************** Dummy Test for Second 485 START *****************************/
#define TAG "RS485_ECHO_APP"
 
// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
  
#define BUF_SIZE        (127)
#define BAUD_RATE       (9600)
 
// Read packet timeout
#define ECHO_TASK_STACK_SIZE    (2048)
#define ECHO_TASK_PRIO          (10)
// #define ECHO_UART_PORT          uart_port_t 2  // (CONFIG_ECHO_UART_PORT_NUM)
 
// Timeout threshold for UART = number of symbols (~10 tics) with unchanged state on receive pin
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
static void echo_send(const int port, const char* str, uint8_t length)
{
    if (uart_write_bytes((uart_port_t)port, str, length) != length) {
        ESP_LOGE(TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}
 
// An example of echo test with hardware flow control on UART
static void echo_task(void *arg)
{
    const uart_port_t uart_num = 2;
    uart_config_t uart_config = {
        .baud_rate = BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 122,
        .source_clk = UART_SCLK_DEFAULT,
    };
 
    // Set UART log level
    esp_log_level_set(TAG, ESP_LOG_INFO);
 
    ESP_LOGI(TAG, "Start RS485 application test and configure UART.");
 
    // Install UART driver (we don't need an event queue here)
    // In this example we don't even use a buffer for sending data.
    ESP_ERROR_CHECK(uart_driver_install(uart_num, BUF_SIZE * 2, 0, 0, NULL, 0));
 
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(uart_num, &uart_config));
 
    ESP_LOGI(TAG, "UART set pins, mode and install driver.");
 
    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(uart_num, RS2_TX, RS2_RX, RS2_DE, -1));
 
    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(uart_num, UART_MODE_RS485_HALF_DUPLEX));
 
    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(uart_num, ECHO_READ_TOUT));
 
    // Allocate buffers for UART
    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
 
    ESP_LOGI(TAG, "UART start recieve loop.\r");
    echo_send(uart_num, "Start RS485 UART test.\r\n", 24);
 
    while (1) {
        //Read data from UART
        int len = uart_read_bytes(uart_num, data, BUF_SIZE, PACKET_READ_TICS);
 
        //Write data back to UART
        if (len > 0) {
            echo_send(uart_num, "\r\n", 2);
            char prefix[] = "RS485 Received: [";
            echo_send(uart_num, prefix, (sizeof(prefix) - 1));
            ESP_LOGI(TAG, "Received %u bytes:", len);
            printf("[ ");
            for (int i = 0; i < len; i++) {
                printf("0x%.2X ", (uint8_t)data[i]);
                echo_send(uart_num, (const char*)&data[i], 1);
                // Add a Newline character if you get a return charater from paste (Paste tests multibyte receipt/buffer)
                if (data[i] == '\r') {
                    echo_send(uart_num, "\n", 1);
                }
            }
            printf("] \n");
            echo_send(uart_num, "]\r\n", 3);
        } else {
            // Echo a "." to show we are alive while we wait for input
            echo_send(uart_num, ".", 1);
            ESP_ERROR_CHECK(uart_wait_tx_done(uart_num, 10));
        }
    }
    vTaskDelete(NULL);
}

void app_main(void) {
	reset_gpio();
	vTaskDelay(pdMS_TO_TICKS(10));
	estop_begin(&estop);
	vTaskDelay(pdMS_TO_TICKS(10));
	bms_485_begin(&bms);
//	battery_ros_init();
	initTwai(CAN1_TX, CAN1_RX);
	setModesOfOperation(1, 3); //set elocity control mode
	setModesOfOperation(2, 3);
	vTaskDelay(pdMS_TO_TICKS(2));
	setDin2Function(1, 0x4000); //set din 2 to activate command
	setDin2Function(2, 0x4000);
	vTaskDelay(pdMS_TO_TICKS(2));
	setProfileSpeed(1, 900); //set rpm for position control mode
	setProfileSpeed(2, 900);
	vTaskDelay(pdMS_TO_TICKS(2));
	setProfileAcceleration(1, 15); //set rps/s for position control mode
	setProfileAcceleration(2, 15);
	vTaskDelay(pdMS_TO_TICKS(2));
	setProfileDeceleration(1, 15); 
	setProfileDeceleration(2, 15);
	vTaskDelay(pdMS_TO_TICKS(2));
	icm42688_spi_begin(&icm42688);
	imu_ros_init();
	amip4k_spi_begin(&amip4k);
	vTaskDelay(pdMS_TO_TICKS(10));
	shoalbot_master_i2c_init(&shoalbot_i2c, &shoalbot_i2c_config);
	shoalbot_master_i2c_begin(&shoalbot_i2c);
//	odom_ros_init();
	amip4k_spi_reset_cnt(&amip4k, 'L');
	amip4k_spi_reset_cnt(&amip4k, 'R');
	left_count_now = 0; left_count_prev = 0; right_count_now = 0; right_count_prev = 0;
	left_counter = 0; left_counter_prev = 0; right_counter = 0; right_counter_prev = 0;
	left_speed_m = 0; right_speed_m = 0;
	x_pos_ = 0.0; y_pos_ = 0.0; heading_ = 0.0;
	vTaskDelay(pdMS_TO_TICKS(10));

	MovingAverageFilter_begin(&left_speed_filter, 1);
	MovingAverageFilter_begin(&right_speed_filter, 1);

	esp_intr_dump(NULL);

	#if defined(RMW_UXRCE_TRANSPORT_CUSTOM)
		rmw_uros_set_custom_transport(true, (void *) &uart_port, esp32_serial_open, esp32_serial_close, esp32_serial_write, esp32_serial_read);
	#else
		#error micro-ROS transports misconfigured
	#endif  // RMW_UXRCE_TRANSPORT_CUSTOM

	xTaskCreate(micro_ros_task, "micro_ros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL);
	//xTaskCreatePinnedToCore(micro_ros_task, "uros_task", CONFIG_MICRO_ROS_APP_STACK, NULL, CONFIG_MICRO_ROS_APP_TASK_PRIO, NULL, 0);
	xTaskCreate(spi_task, "spi_task", 16000, NULL, 5, NULL);
	// //xTaskCreatePinnedToCore(spi_task, "spi_task", 16000, NULL, 5, NULL, 1);
	xTaskCreate(rs485_task, "rs485_task", 16000, NULL, 5, NULL);
	// //xTaskCreatePinnedToCore(rs485_task, "rs485_task", 16000, NULL, 5, NULL, 1);
	xTaskCreate(twai_task, "twai_task", 16000, NULL, 5, NULL);
	// //xTaskCreatePinnedToCore(twai_task, "twai_task", 16000, NULL, 5, NULL, 1);
	xTaskCreate(i2c_task, "i2c_task", 16000, NULL, 5, NULL);
	// //xTaskCreatePinnedToCore(i2c_task, "i2c_task", 16000,  NULL, 5, NULL, 0);
	xTaskCreate(set_DO, "set_DO_task", 16000, NULL, 5, NULL);
	xTaskCreate(echo_task, "uart_echo_task", ECHO_TASK_STACK_SIZE, NULL, ECHO_TASK_PRIO, NULL);

}