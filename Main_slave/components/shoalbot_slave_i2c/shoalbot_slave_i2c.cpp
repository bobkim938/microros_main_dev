#include "shoalbot_slave_i2c.h"

shoalbot_slave_i2c::shoalbot_slave_i2c(i2c_slave_config* slave_config) {
    slv_conf.i2c_port = I2C_NUM_0;
    slv_conf.sda_io_num = slave_config->sda;
    slv_conf.scl_io_num = slave_config->scl;
    slv_conf.clk_source = I2C_CLK_SRC_DEFAULT;
    slv_conf.send_buf_depth = 256;
    slv_conf.slave_addr = slave_config->slaveAddr;
    slv_conf.addr_bit_len = I2C_ADDR_BIT_LEN_7;

    gpio_set_pull_mode(slave_config->sda, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(slave_config->scl, GPIO_PULLUP_ONLY);

    ESP_ERROR_CHECK(i2c_new_slave_device(&slv_conf, &slv_handle));
}

esp_err_t shoalbot_slave_i2c::i2c_read() {
    uint8_t* data_rcv = (uint8_t *)(malloc(sizeof(uint8_t)));
    *data_rcv = 0;
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));
    if (receive_queue == NULL) {
        free(data_rcv); 
        //ESP_LOGE("I2C", "Failed to create queue");
        return 0; 
    }

    cbs.on_recv_done = i2c_slave_rx_done_callback;
    ESP_ERROR_CHECK(i2c_slave_register_event_callbacks(slv_handle, &cbs, receive_queue));
    ESP_ERROR_CHECK(i2c_slave_receive(slv_handle, data_rcv, 10));
    
    i2c_slave_rx_done_event_data_t rx_data;
    if(DO_cnt == 0) {
        parsed_data_DO = 0;
        parsed_data_NavState = 0;
    }
    else if(BMS_cnt == 0) {
        parsed_data_BMS = 0;
    }

    if (xQueueReceive(receive_queue, &rx_data, 10) == pdPASS) {
        if(DO_ack_flag && DO_cnt == 0) {
            // ESP_LOGI("I2C", "Data received 1: %d", *data_rcv);
            parsed_data_DO |= (*data_rcv << 8);
            DO_cnt++;
        }
        else if(DO_ack_flag && DO_cnt == 1) {
            // ESP_LOGI("I2C", "Data received 2: %d", *data_rcv);
            parsed_data_DO |= (*data_rcv);
            DO_cnt++;
        }
        else if(DO_ack_flag && DO_cnt == 2) {
            // ESP_LOGI("I2C", "Data received 3: %d", *data_rcv);
            parsed_data_NavState |= (*data_rcv << 8);
            DO_cnt++;
        }
        else if(DO_ack_flag && DO_cnt == 3) {
            // ESP_LOGI("I2C", "Data received 4: %d", *data_rcv);
            parsed_data_NavState |= *data_rcv;
            DO_cnt = 0;
            DO_ack_flag = false;
            new_DO = true;
            free(data_rcv); // free allocated memory
            vQueueDelete(receive_queue); // delete the queue to free resources
            return ret;
        }
        else if(State_ack_flag) {
            i2c_send_state(current_State, 6);
            State_ack_flag = false;
        }
        else if(BMS_ack_flag && BMS_cnt == 0) {
            // ESP_LOGI("I2C", "Data received 1: %d", *data_rcv);
            parsed_data_BMS = (*data_rcv << 24); 
            BMS_cnt++;
        }
        else if(BMS_ack_flag && BMS_cnt == 1) {
            // ESP_LOGI("I2C", "Data received 2: %d", *data_rcv); 
            parsed_data_BMS |= (*data_rcv << 16); 
            BMS_cnt++;
        }
        else if(BMS_ack_flag && BMS_cnt == 2) {
            // ESP_LOGI("I2C", "Data received 3: %d", *data_rcv);
            parsed_data_BMS |= (*data_rcv << 8); 
            BMS_cnt++;
        }
        else if(BMS_ack_flag && BMS_cnt == 3) {
            // ESP_LOGI("I2C", "Data received 4: %d", *data_rcv);
            parsed_data_BMS |= *data_rcv; 
            BMS_cnt = 0;
            BMS_ack_flag = false;
            new_BMS = true;
            current_BMS = parsed_data_BMS;
            printf("Battery Percentage: %lu\n", parsed_data_BMS);
            free(data_rcv); // free allocated memory
            vQueueDelete(receive_queue); // delete the queue to free resources
            return ret;
        }
        else if(batSW_ack_flag && *data_rcv == 0xAA) {
            //printf("batSW received: %d\n", *data_rcv);
            batSW_ack_flag = false;
            new_batSW = true;
            free(data_rcv); // free allocated memory
            vQueueDelete(receive_queue); // delete the queue to free resources
            return ret;
        }
        else if(batSW_ack_flag) {
            ret = i2c_slave_transmit(slv_handle, &batSW, 1, -1);
            //printf("batSW sent: %d\n", batSW);
            if(ret != ESP_OK) {
                //ESP_LOGE(TAG, "Failed to transmit data");
            }
            batSW_ack_flag = false;
        }

        if(*data_rcv == 0xBB && !DO_ack_flag && !State_ack_flag && !BMS_ack_flag && !batSW_ack_flag) {
            DO_ack_flag = true;
            // printf("DO acknowlodged\n");
        }
        else if(*data_rcv == 0xFF && !State_ack_flag && !DO_ack_flag && !BMS_ack_flag && !batSW_ack_flag) {
            State_ack_flag = true;
            new_State = true;
            // printf("DI acknowlodged\n");
        }
        else if(*data_rcv == 0xCC && !State_ack_flag && !DO_ack_flag && !BMS_ack_flag && !batSW_ack_flag) {
            BMS_ack_flag = true;
            // printf("BMS acknowlodged\n");
        }
        else if(*data_rcv == 0xDD && !State_ack_flag && !DO_ack_flag && !BMS_ack_flag && !batSW_ack_flag) {
            batSW_ack_flag = true;
            new_batSW = true;
            // printf("batSW acknowlodged\n");
        }
    } else {
        ESP_LOGE("I2C", "Failed to receive data");
    }

    free(data_rcv); // free allocated memory
    vQueueDelete(receive_queue); // delete the queue to free resources

    return ret;
}

esp_err_t shoalbot_slave_i2c::i2c_send_state(uint8_t* data, uint8_t index) {
    for(int i = 0; i < index; i++) {
        ret = i2c_slave_transmit(slv_handle, data + i, 1, -1);
        // printf("DI sent: %d\n", *(data + i));
        if(ret != ESP_OK) {
                //ESP_LOGE(TAG, "Failed to transmit data");
                return ret;
            }
    }
    // printf("DI sent\n");
    return ret;
}

bool shoalbot_slave_i2c::get_state() {
    if(new_State) {
        new_State = false;
        return true;
    }
    else {
        return false;
    }
}

bool shoalbot_slave_i2c::get_do() {
    if(new_DO) {
        new_DO = false;
        return true;
    }
    else {
        return false;
    }
}

bool shoalbot_slave_i2c::get_bms() {
    if(new_BMS) {
        new_BMS = false;
        return true;
    }
    else {
        return false;
    }
}

uint32_t shoalbot_slave_i2c::return_BMS() {
    return current_BMS;
}

uint16_t shoalbot_slave_i2c::return_NavState() {
    return parsed_data_NavState;
}

uint16_t shoalbot_slave_i2c::return_DO() {
    return parsed_data_DO;
}
/*
bool shoalbot_slave_i2c::get_batSW() {
    if(new_batSW) {
        new_batSW = false;
        return true;
    }
    else {
        return false;
    }
}
*/
void shoalbot_slave_i2c::set_state(uint8_t* state_data) {
    for(int i = 0; i < 6; i++) {
        current_State[i] = *(state_data + i);
    }
}
/*
void shoalbot_slave_i2c::set_batSW(bool batsw) {
    if(batsw) {
        this->batSW = 1;
        printf("batSW On\n");
    }
    else {
        this->batSW = 0;
    }
}
*/