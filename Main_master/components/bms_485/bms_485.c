/**
 * This file contains the implementation of the bms_485 class.
 *
 * The bms_485 class is used to communicate with the BMS system in half duplex mode.
 * It initializes the UART communication, also contains the implementation of methods 
 * to get the BMS data, get voltage, get current, get charge, get capacity, etc.
 * 
 * @file bms_485.cpp
 */

#include <stdio.h>
#include "bms_485.h"

/**
 * Initializes the bms_485 object by setting up the UART communication.
 * 
 * This function sets up the UART driver, configures the UART parameters, 
 * sets the UART pins, sets the RS485 half duplex mode, and sets the read 
 * timeout of the UART TOUT feature. It also sends a command to the BMS 
 * system to initialize the communication.
 * 
 * @return None.
 */


void bms_485_begin(shoalbot_bms* obj){
    obj->bms_uart_num = 1;
    obj->uart_config.baud_rate = BMS_BAUD_RATE;
    obj->uart_config.data_bits = UART_DATA_8_BITS;
    obj->uart_config.parity = UART_PARITY_DISABLE;
    obj->uart_config.stop_bits = UART_STOP_BITS_1;
    obj->uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    obj->uart_config.rx_flow_ctrl_thresh = 122;
    obj->uart_config.source_clk = UART_SCLK_APB;

    // Set UART log level
    esp_log_level_set(BMS_TAG, ESP_LOG_INFO);

    ESP_LOGI(BMS_TAG, "Start RS485 application test and configure UART.");

    // Install UART driver (we don't need an event queue here)
    ESP_ERROR_CHECK(uart_driver_install(obj->bms_uart_num, BMS_BUF_SIZE * 2, 0, 0, NULL, 0));
        
    // Configure UART parameters
    ESP_ERROR_CHECK(uart_param_config(obj->bms_uart_num, &obj->uart_config));

    // Set UART pins as per KConfig settings
    ESP_ERROR_CHECK(uart_set_pin(obj->bms_uart_num, BMS_TEST_TXD, BMS_TEST_RXD, BMS_TEST_RTS, BMS_TEST_CTS));

    // Set RS485 half duplex mode
    ESP_ERROR_CHECK(uart_set_mode(obj->bms_uart_num, UART_MODE_RS485_HALF_DUPLEX));

    // Set read timeout of UART TOUT feature
    ESP_ERROR_CHECK(uart_set_rx_timeout(obj->bms_uart_num, ECHO_READ_TOUT));

    gpio_reset_pin(BMS_TEST_RTS);
    gpio_set_direction(BMS_TEST_RTS, GPIO_MODE_OUTPUT);
    gpio_set_level(BMS_TEST_RTS, 1);
    bms_485_bms_echo_send(obj->bms_uart_num, "\r\n", 2);
}

/**
 * Retrieves BMS data from the UART and decodes various values based on the received data.
 *
 * @throws None
 */
void bms_485_getBMSData(shoalbot_bms* obj){
    // Allocate buffers for UART
    vTaskDelay(2000/portTICK_PERIOD_MS);
    // uint8_t data[BMS_BUF_SIZE];
    int len = 0;  // Number of bytes received
    uint8_t buf = 0;
    while (len < 120){
        // Write data over UART to BMS.
        uint8_t data[BMS_BUF_SIZE];
        gpio_set_level(BMS_TEST_RTS, 1);
        bms_485_bms_echo_send(obj->bms_uart_num, "\r\n", 2);
        vTaskDelay(10/portTICK_PERIOD_MS);
        //Read data from UART
        gpio_set_level(BMS_TEST_RTS, 0);
        len = uart_read_bytes(obj->bms_uart_num, data, BMS_BUF_SIZE, PACKET_READ_TICS);
        vTaskDelay(200/portTICK_PERIOD_MS);
        // The header of the pakcet will be 0x7E, 0X32.
        // So check the header first.
        while (data[buf] != 126 || data[buf+1] != 50) {
            buf++;
            if (buf > 100)  return;
        }
        // Ignore some data in front.  
        buf += (uint8_t)BUFFER_BIAS;
        /* CALCULATE BY BYTES START*/
        // Temperature should minus 40 to convert to degree, according to the datasheet.
        obj->_cell_temperature[0] = (bms_485_hex_to_dec(data[buf]) << 12) + (bms_485_hex_to_dec(data[buf + 1]) << 8)
                    + (bms_485_hex_to_dec(data[buf + 2]) << 4) + (bms_485_hex_to_dec(data[buf + 3])) - 40;
        obj->_cell_temperature[1] = (bms_485_hex_to_dec(data[buf + 4]) << 12) + (bms_485_hex_to_dec(data[buf + 5]) << 8)
                    + (bms_485_hex_to_dec(data[buf + 6]) << 4) + (bms_485_hex_to_dec(data[buf + 7])) - 40;
        obj->_cell_temperature[2] = (bms_485_hex_to_dec(data[buf + 8]) << 12) + (bms_485_hex_to_dec(data[buf + 9]) << 8)
                            + (bms_485_hex_to_dec(data[buf + 10]) << 4) + (bms_485_hex_to_dec(data[buf + 11])) - 40;
        // Cast to int16_t to Calculate current, which can be negative, unit is 10 mA.
        obj->_current = (int16_t)((bms_485_hex_to_dec(data[buf + 12]) << 12) + (bms_485_hex_to_dec(data[buf + 13]) << 8)
                            + (bms_485_hex_to_dec(data[buf + 14]) << 4) + bms_485_hex_to_dec(data[buf + 15]));
        // _current = (int_16_t)((0x0000 | (data[buf + 12] << 12) | (data[buf + 13] << 8) | (data[buf + 14] << 4) | data[buf + 15]));
        // Cast to int16_t to Calculate voltage, which can be negative, unit is 1 mV.
        // if (_current > 0x7FFFFFFF) _current -= 0x100000000;
        obj->_voltage = (bms_485_hex_to_dec(data[buf + 16]) << 12) + (bms_485_hex_to_dec(data[buf + 17]) << 8)
                            + (bms_485_hex_to_dec(data[buf + 18]) << 4) + (bms_485_hex_to_dec(data[buf + 19]));
        obj->_charge = (bms_485_hex_to_dec(data[buf + 20]) << 12) + (bms_485_hex_to_dec(data[buf + 21]) << 8)
        
                            + (bms_485_hex_to_dec(data[buf + 22]) << 4) + (bms_485_hex_to_dec(data[buf + 23]));
        obj->_capacity = (bms_485_hex_to_dec(data[buf + 26]) << 12) + (bms_485_hex_to_dec(data[buf + 27]) << 8)
                            + (bms_485_hex_to_dec(data[buf + 28]) << 4) + (bms_485_hex_to_dec(data[buf + 29]));
        obj->_cycle = (bms_485_hex_to_dec(data[buf + 30]) << 12) + (bms_485_hex_to_dec(data[buf + 31]) << 8)
                            + (bms_485_hex_to_dec(data[buf + 32]) << 4) + (bms_485_hex_to_dec(data[buf + 33]));
        /* CALCULATE BY BYTES END*/
        // _current = (hex_to_dec(data[buf + 12]));
        // _voltage = (hex_to_dec(data[buf + 13]));

    }
}

/**
 * Sends data over UART and handles sending failure.
 *
 * @param port the UART port number
 * @param str pointer to the data buffer to be sent
 * @param length the length of the data to be sent
 *
 * @throws None
 */
void bms_485_bms_echo_send(uart_port_t port, const char* str, uint8_t length)
{
    if (uart_write_bytes(port, str, length) != length) {
        ESP_LOGE(BMS_TAG, "Send data critical failure.");
        // add your code to handle sending failure here
        abort();
    }
}

/**
 * Converts a hexadecimal value to its decimal equivalent.
 *
 * @param val the hexadecimal value to be converted
 *
 * @return the decimal equivalent of the input value
 *
 * @throws None
 */
uint8_t bms_485_hex_to_dec(uint8_t val) {
    if(val >= '0' && val <= '9')
    {
        val = val - 48;
    }
    else if(val >= 'A' && val <= 'F')
    {
        val = val - 65 + 10;
    }
    return val;
}

float_t bms_485_getTemperature(shoalbot_bms* obj) {
    obj->_temperature = (obj->_cell_temperature[0] + obj->_cell_temperature[1] + obj->_cell_temperature[2]) / 3.0;
    return obj->_temperature;
}

float_t bms_485_getVoltage(shoalbot_bms* obj) {
    return obj->_voltage * 0.001;
}

float_t bms_485_getCurrent(shoalbot_bms* obj) {
    return obj->_current * 0.01;
}

float_t bms_485_getCharge(shoalbot_bms* obj){
    return obj->_charge;
}

float_t bms_485_getCapacity(shoalbot_bms* obj) {
    return obj->_capacity;
}

float_t bms_485_getDesignCapacity(shoalbot_bms* obj) {
    return obj->_design_capacity;
}

float_t bms_485_getPercentage(shoalbot_bms* obj) {
    return obj->_percentage;
}

float_t* bms_485_getCellTemperature(shoalbot_bms* obj) {
    return obj->_cell_temperature;
}

uint16_t bms_485_getCycle(shoalbot_bms* obj) {
    return obj->_cycle;
}