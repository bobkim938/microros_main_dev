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

uint32_t i2c_slave::i2c_read() {
    if(DO_cnt == 0) {
        parsed_data = 0;
    }
    uint8_t* data_rcv = (uint8_t *)(malloc(sizeof(uint8_t)));
    *data_rcv = 0;
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));
    if (receive_queue == NULL) {
        free(data_rcv); // Ensure memory is freed if queue creation fails
        ESP_LOGE("I2C", "Failed to create queue");
        return 0; // Consider an error code or exception
    }

    cbs.on_recv_done = i2c_slave_rx_done_callback;
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slv_handle, &cbs, receive_queue));
    ESP_ERROR_CHECK(i2c_slave_receive(slv_handle, data_rcv, 10));

    i2c_slave_rx_done_event_data_t rx_data;
    if (xQueueReceive(receive_queue, &rx_data, 10) == pdPASS) {
        if(DO_cnt == 0) {
            ESP_LOGI("I2C", "Data received 1: %d", *data_rcv);
            parsed_data |= (*data_rcv << 16);
            DO_cnt++;
        }
        else if(DO_cnt == 1) {
            ESP_LOGI("I2C", "Data received 2: %d", *data_rcv);
            parsed_data |= (*data_rcv << 8);
            DO_cnt++;
        }
        else if(DO_cnt == 2) {
            ESP_LOGI("I2C", "Data received 3: %d", *data_rcv);
            parsed_data |= *data_rcv;
            DO_cnt = 0;
            printf("Parsed Data: 0x%04lX\n", parsed_data);
            free(data_rcv); // Free allocated memory
            vQueueDelete(receive_queue); // Delete the queue to free resources
            return parsed_data;
        }
    } else {
        ESP_LOGE("I2C", "Failed to receive data");
    }

    free(data_rcv); // Free allocated memory
    vQueueDelete(receive_queue); // Delete the queue to free resources

    return 0;
}

