#ifndef BMS_485_H
#define BMS_485_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <math.h>

// #include </home/astri/microros_main_dev/Main_slave/components/micro_ros_espidf_component/include/rcl/rcl/rcl.h>
// #include </home/astri/microros_main_dev/Main_slave/components/micro_ros_espidf_component/include/rcl/rcl/init.h>
// #include <rcl/error_handling.h>

/**e
 * This is a test code to connect with the BMS system in half duplex mode.
*/
#define BMS_TAG "BATTERY_PORT"

// Note: Some pins on target chip cannot be assigned for UART communication.
// Please refer to documentation for selected board and target to configure pins using Kconfig.
#define BMS_TEST_TXD   (GPIO_NUM_17) //(37) // (CONFIG_ECHO_UART_TXD)
#define BMS_TEST_RXD   (GPIO_NUM_18) //(38) // (CONFIG_ECHO_UART_RXD)

// RTS for RS485 Half-Duplex Mode manages DE/~RE
#define BMS_TEST_RTS   (GPIO_NUM_8)//(39) // (CONFIG_ECHO_UART_RTS)

// CTS is not used in RS485 Half-Duplex Mode
#define BMS_TEST_CTS   (UART_PIN_NO_CHANGE)

#define BMS_BUF_SIZE        (260)  // should be no more than 130
#define BMS_BAUD_RATE       (9600)

#define BUFFER_BIAS       (73) //(85)

// #define BMS_UART_PORT          uart_port_t(1)  // (CONFIG_ECHO_UART_PORT_NUM)
#define ECHO_READ_TOUT          (3) // 3.5T * 8 = 28 ticks, TOUT=3 -> ~24..33 ticks
#define BMS_TASK_STACK_SIZE     (16000)  // (9192) CONFIG_MICRO_ROS_APP_STACK
#define BMS_TASK_PRIO          (10)
#define PACKET_READ_TICS        (200)  // 100

void bms_485_begin();
void bms_485_getBMSData();
// void getBmsData(float_t* current, float_t* voltage, float_t* charge, float_t* capacity, uint8_t* cycle);
float_t bms_485_getTemperature();
float_t bms_485_getVoltage();
float_t bms_485_getCurrent();
float_t bms_485_getCharge();
float_t bms_485_getCapacity();
float_t bms_485_getDesignCapacity();
float_t bms_485_getPercentage();
float_t* bms_485_getCellTemperature();
uint16_t bms_485_getCycle();
void bms_485_bms_echo_send(uart_port_t port, const char* str, uint8_t length);
uint8_t bms_485_hex_to_dec(uint8_t val);  // Converts a hexadecimal value to its decimal equivalent.


#endif // BMS_485_H