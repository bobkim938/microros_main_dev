#include "i2c_master.h"

i2c_master::i2c_master(i2c_master_config* conf) {
    i2c_mst_config.sda_io_num = conf -> sda;
    i2c_mst_config.scl_io_num = conf -> scl;
    i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    i2c_mst_config.i2c_port = I2C_NUM_0;
    i2c_mst_config.glitch_ignore_cnt = 7;
    i2c_mst_config.flags.enable_internal_pullup = true;
    i2c_mst_config.trans_queue_depth = 10;

    ESP_slave.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    ESP_slave.device_address = conf -> slaveAddr;
    ESP_slave.scl_speed_hz = 100000; 

    gpio_set_pull_mode(conf->sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(conf->scl, GPIO_PULLUP_ONLY);
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

esp_err_t i2c_master::i2c_send_DO(uint8_t* data, uint8_t index) { // first byte 0xBB for DO cmd
    for(int i = 0; i < index; i++) {
        ret = i2c_master_transmit(i2c_master_handle, data + i, sizeof(data + i), -1);
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to transmit data");
            return ret;
        }
    }
    return ret;
}

// uint32_t i2c_master::i2c_read_DI() {
//     if (ret != ESP_OK) {
//         ESP_LOGE(TAG, "Failed to transmit read DI command");
//         return 0;
//     }
//     uint8_t* data_rcv = (uint8_t *)(malloc(sizeof(uint8_t)));
 
//     QueueHandle_t receive_queue = xQueueCreate(1, sizeof(i2c_master_event_data_t));
//     if (receive_queue == NULL) {
//         free(data_rcv);
//         ESP_LOGE("I2C", "Failed to create queue");
//         return 0;
//     }
 
//     cbs.on_trans_done = i2c_master_rx_done_callback;
//     ESP_ERROR_CHECK(i2c_master_register_event_callbacks(i2c_master_handle, &cbs, receive_queue));
//     ESP_ERROR_CHECK(i2c_master_receive(i2c_master_handle, data_rcv, 1, -1));
 
//     i2c_master_event_data_t rx_data;
 
//    if (diCnt == 0) {
//         DI_fromSlave = 0;
//         esp_err_t ret = i2c_master_transmit(i2c_master_handle, DI_cmd, sizeof(DI_cmd), portMAX_DELAY); // Consider using a specific timeout instead of portMAX_DELAY
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG, "Failed to transmit read DI command");
//             return 0;
//         }
//         diCnt++;
//     }      
 
//     if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(100)) == pdPASS) {
//         if(diCnt == 1) {
//             ESP_LOGI("I2C", "Data received 1: %d", *data_rcv);
//             DI_fromSlave |= ((uint32_t)(*data_rcv)) << 16;
//             diCnt++;
//         }
//         else if(diCnt == 2) {
//             ESP_LOGI("I2C", "Data received 2: %d", *data_rcv);
//             DI_fromSlave |= ((uint32_t)(*data_rcv)) << 8;
//             diCnt++;
//         }
//         else if(diCnt == 3) {
//             ESP_LOGI("I2C", "Data received 3: %d", *data_rcv);
//             DI_fromSlave |= ((uint32_t)(*data_rcv));
//             diCnt = 0;
//             printf("DI from slave: 0x%06lX\n", DI_fromSlave);
//             free(data_rcv);
//             vQueueDelete(receive_queue);
//             return DI_fromSlave;
//         }
//     }
//     else {
//         ESP_LOGE("I2C", "Failed to receive data");
//         return 0;
//     }
//     free(data_rcv); // free allocated memory
//     vQueueDelete(receive_queue); // delete the queue to free resources
 
//     return ESP_OK;
// }

esp_err_t i2c_master::read_di() {
    i2c_master_transmit_receive(i2c_master_handle, DI_cmd, sizeof(DI_cmd), DI_data, 3, -1);
    ESP_LOGI("I2C", "Data received 1: %d", DI_data[0]);
    ESP_LOGI("I2C", "Data received 2: %d", DI_data[1]);
    ESP_LOGI("I2C", "Data received 3: %d", DI_data[2]);
    return ESP_OK;
}

