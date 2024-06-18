#include "i2c_slave.h"

i2c_slave::i2c_slave(i2c_slave_config* slave_config) {
    slv_conf.i2c_port = I2C_NUM_0;
    slv_conf.sda_io_num = slave_config->sda;
    slv_conf.scl_io_num = slave_config->scl;
    slv_conf.clk_source = I2C_CLK_SRC_DEFAULT;
    slv_conf.send_buf_depth = 256;
    slv_conf.addr_bit_len = I2C_ADDR_BIT_LEN_7;
    slv_conf.slave_addr = slave_config->slaveAddr;

    gpio_set_pull_mode(slave_config->sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(slave_config->scl, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(i2c_new_slave_device(&slv_conf, &slv_handle));
}

uint8_t i2c_slave::i2c_read() {
    uint8_t* data_rcv = (uint8_t *)(malloc(sizeof(uint8_t)));
    *data_rcv = 0;
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t)); // maximum of 1 item in the queue
    cbs.on_recv_done = i2c_slave_rx_done_callback;
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slv_handle, &cbs, receive_queue));
    ESP_ERROR_CHECK(i2c_slave_receive(slv_handle, data_rcv, 10)); 
    xQueueReceive(receive_queue, &rx_data, 10); 
    ESP_LOGI("I2C", "Data received: %d", *data_rcv);

    return *data_rcv;
}