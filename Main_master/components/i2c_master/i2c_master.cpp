#include "i2c_master.h"

i2c_master::i2c_master(i2c_master_config* conf) {
    i2c_mst_config.sda_io_num = conf -> sda;
    i2c_mst_config.scl_io_num = conf -> scl;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_mst_config, &i2c_mst_handle));
}

esp_err_t i2c_master::add_slave_device(uint16_t slave_addr) {
    ESP_slave.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    ESP_slave.device_address = slave_addr;
    ESP_slave.scl_speed_hz = 100000; 
    esp_err_t ret = i2c_master_bus_add_device(i2c_mst_handle, &ESP_slave, &i2c_slave_handle);
    return ret;
}

