#include "i2c_master.h"

i2c_master::i2c_master(i2c_master_config* conf) {
    i2c_mst_config.sda_io_num = conf -> sda;
    i2c_mst_config.scl_io_num = conf -> scl;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_slave.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    ESP_slave.device_address = conf -> slaveAddr;
    ESP_slave.scl_speed_hz = 100000; 
}

esp_err_t i2c_master::begin() {
    ret = i2c_new_master_bus(&i2c_mst_config, &i2c_mst_handle);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus");
        return ret;
    }
    ret = i2c_master_bus_add_device(i2c_mst_handle, &ESP_slave, &i2c_master_handle); 
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add slave device");
        return ret;
    }
    return ret;
}

esp_err_t i2c_master::i2c_send_DO(uint8_t* data) {
    for(int i = 0; i < 3; i++) {
        ret = i2c_master_transmit(i2c_master_handle, data + i, sizeof(data + i), -1);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit data");
            return ret;
        }
    }
    return ret;
}

esp_err_t i2c_master::i2c_read_DI() {
    ret = i2c_master_receive(i2c_master_handle, DI_data, sizeof(DI_data), -1);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive data");
        return ret;
    }
    ret = i2c_master_receive(i2c_master_handle, DI_data + 1, sizeof(DI_data + 1), -1);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive data");
        return ret;
    }
    ret = i2c_master_receive(i2c_master_handle, DI_data + 2, sizeof(DI_data + 2), -1);
    if(ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to receive data");
        return ret;
    }
    return ret;
}