#include "i2c_slave.h"

i2c_slave::i2c_slave(i2c_slave_config* slave_config) {
    conf_slave.mode = I2C_MODE_SLAVE;
    conf_slave.sda_io_num = slave_config -> sda;
    conf_slave.scl_io_num = slave_config -> scl;
    conf_slave.slave.slave_addr = slave_config -> slaveAddr;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.slave.addr_10bit_en = 0; // 7-bit slave address
    conf_slave.clk_flags = 0;
}   

esp_err_t i2c_slave::begin() {
    esp_err_t err = i2c_param_config(i2c_port, &conf_slave);
    if(err != ESP_OK) {
        return err;
    }
    err = i2c_driver_install(i2c_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
    if(err != ESP_OK) {
        return err;
    }
    return ESP_OK;
}

uint8_t i2c_slave::slave_read_buffer() {
    int bytes_received = i2c_slave_read_buffer(I2C_NUM_0, pong, I2C_SLAVE_RX_BUF_LEN, 100 / portTICK_PERIOD_MS);
    int i = 0;
    ESP_LOGI(TAG, "Bytes Received: %d", bytes_received);
    if(bytes_received > 0) {
        while(i < bytes_received) { // Corrected loop condition
            if(pong[i] == 1) {
                break;
            }
            i++;
        }
        ESP_LOGI(TAG, "Received Index: %d", i);
    }
    return i;
}



