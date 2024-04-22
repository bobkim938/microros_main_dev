
#include "i2c_master.h"

i2c_master::i2c_master(i2c_master_config* master_config) {
    this->master_config.mode = I2C_MODE_MASTER;
    this->master_config.sda_io_num = master_config -> sda;
    this->master_config.scl_io_num = master_config -> scl;
    this->master_config.sda_pullup_en = GPIO_PULLUP_ENABLE;
    this->master_config.scl_pullup_en = GPIO_PULLUP_ENABLE;
    this->master_config.master.clk_speed = 100000;
    slave_addr = master_config -> slaveESP_addr;
}

esp_err_t i2c_master::begin() {
    esp_err_t ret = i2c_param_config(i2c_master_port, &this->master_config);
    if (ret != ESP_OK) {
        return ret;
    }
    ESP_LOGI("I2C_MASTER", "I2C master configuration set");
    return i2c_driver_install(i2c_master_port, this->master_config.mode, 0, 0, 0);
}

esp_err_t i2c_master::i2c_master_send(uint8_t* msg, size_t size) {
    ESP_LOGI(TAG, "Bytes to be sent to slave = %d", size);  
    esp_err_t ret; 
    cmd = i2c_cmd_link_create(); // create i2c command handle    
    i2c_master_start(cmd); // add start and stop conditions to the I2C command sequence
    i2c_master_write_byte(cmd, slave_addr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN); // add write commands to the I2C command sequence
    i2c_master_write(cmd, msg, size, ACK_CHECK_EN); // master write buffer to the i2c bus
    i2c_master_stop(cmd);
    
    ret = i2c_master_cmd_begin(i2c_master_port, cmd, 100 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}