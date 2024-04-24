#include "i2c_slave.h"

i2c_slave::i2c_slave(i2c_slave_config* slave_config) {
    slv_conf.i2c_port = I2C_NUM_0;
    slv_conf.sda_io_num = slave_config->sda;
    slv_conf.scl_io_num = slave_config->scl;
    slv_conf.clk_source = I2C_CLK_SRC_DEFAULT;
    slv_conf.send_buf_depth = 256;
    slv_conf.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    slv_conf.slave_addr = slave_config->slaveAddr;

    ESP_ERROR_CHECK(i2c_new_slave_device(&slv_conf, &slv_handle));
}