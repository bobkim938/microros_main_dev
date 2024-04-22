#include "i2c_slave.h"

i2c_slave::i2c_slave(i2c_slave_config* slave_config) {
    conf_slave.sda_io_num = slave_config -> sda;
    conf_slave.scl_io_num = slave_config -> scl;
    conf_slave.slave.slave_addr = slave_config -> slaveAddr;
    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf_slave.mode = I2C_MODE_SLAVE;
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
    uint8_t data[I2C_SLAVE_RX_BUF_LEN];
    int bytes_received = i2c_slave_read_buffer(i2c_port, data, I2C_SLAVE_RX_BUF_LEN, 100);
    int i = 0;
    if(bytes_received > 0) {
        for(i = 0; i < bytes_received; i++) {
            if(data[i] == 1) break;
        }
        if(i - 1 == 0) {
            return i; // return array index
        }
    }
    return 0;
}

esp_err_t i2c_slave::slave_write_buffer(double* data, size_t array_size) {
    size_t buffer_size = sizeof(double) * array_size;
    uint8_t* buffer = (uint8_t*)malloc(buffer_size);
    if (buffer == NULL) {
        return ESP_ERR_NO_MEM;
    }
    memcpy(buffer, data, buffer_size);
    esp_err_t err = i2c_slave_write_buffer(I2C_NUM_0, buffer, buffer_size, portMAX_DELAY);
    free(buffer);
    return err;
}


