#include "shoalbot_master_i2c.h"

void shoalbot_master_i2c_init(shoalbot_master_i2c* obj, i2c_master_config* conf) {
    obj->i2c_mst_config.sda_io_num = conf->sda;
    obj->i2c_mst_config.scl_io_num = conf->scl;
    obj->i2c_mst_config.clk_source = I2C_CLK_SRC_DEFAULT;
    obj->i2c_mst_config.i2c_port = I2C_NUM_0;
    obj->i2c_mst_config.glitch_ignore_cnt = 7;
    obj->i2c_mst_config.flags.enable_internal_pullup = true;
    obj->i2c_mst_config.trans_queue_depth = 10;

    obj->ESP_slave.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    obj->ESP_slave.device_address = conf->slaveAddr;
    obj->ESP_slave.scl_speed_hz = 100000; 

    obj->DI_cmd[0] = 0xFF;
    obj->batSW_cmd[0] = 0xDD;

    // gpio_set_pull_mode(conf->sda, GPIO_PULLUP_ONLY);
    // gpio_set_pull_mode(conf->scl, GPIO_PULLUP_ONLY);
}

esp_err_t shoalbot_master_i2c_begin(shoalbot_master_i2c* obj) {
    obj->ret = i2c_new_master_bus(&obj->i2c_mst_config, &obj->i2c_mst_handle);
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to create I2C master bus");
        return obj->ret;
    }
    obj->ret = i2c_master_bus_add_device(obj->i2c_mst_handle, &obj->ESP_slave, &obj->i2c_master_handle); 
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to add slave device");
        return obj->ret;
    }
    return obj->ret;
}

esp_err_t shoalbot_master_i2c_i2c_send_DO(shoalbot_master_i2c* obj, uint8_t* data) { 
    obj->ret = i2c_master_transmit(obj->i2c_master_handle, data, 1, 10);
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return obj->ret;
    }
    vTaskDelay(1);
    obj->ret = i2c_master_transmit(obj->i2c_master_handle, data + 1, 1, 10);
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return obj->ret;
    }
    vTaskDelay(1);
    obj->ret = i2c_master_transmit(obj->i2c_master_handle, data + 2, 1, 10);
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return obj->ret;
    }
    vTaskDelay(1);
    obj->ret = i2c_master_transmit(obj->i2c_master_handle, data + 3, 1, 10);
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return obj->ret;
    }
    vTaskDelay(1);
    obj->ret = i2c_master_transmit(obj->i2c_master_handle, data + 4, 1, 10);
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to transmit data");
        return obj->ret;
    }
    return obj->ret;
}

uint32_t shoalbot_master_i2c_read_state(shoalbot_master_i2c* obj) {
    obj->DI_fromSlave = 0;
    obj->ret = i2c_master_transmit_receive(obj->i2c_master_handle, obj->DI_cmd, sizeof(obj->DI_cmd), obj->state_data, 6, -1);
    if(obj->ret != ESP_OK) {
        //ESP_LOGE(TAG, "Failed to retrieve DI");
        return obj->ret;
    }
    obj->DI_fromSlave = ((uint32_t)obj->state_data[0]) << 16 | ((uint32_t)obj->state_data[1]) << 8 | ((uint32_t)obj->state_data[2]);
    obj->DO_slave[0] = obj->state_data[3];
    obj->DO_slave[1] = 0x03 & obj->state_data[4];
    
    // obj->AMR_state[1] = obj->state_data[4] >> 2;
    obj->AMR_state[0] = obj->state_data[5];

    ESP_LOGI("I2C", "DI: 0x%08X\n", (unsigned int)(obj->DI_fromSlave));
    ESP_LOGI("I2C", "DO: 0x%02X\n", obj->DO_slave[0]);
    ESP_LOGI("I2C", "DO1: 0x%02X\n", obj->DO_slave[1]);
    ESP_LOGI("I2C", "AMR: 0x%02X\n", obj->AMR_state[0]);
    ESP_LOGI("I2C", "AMR1: 0x%02X\n", obj->AMR_state[1]);

    return obj->DI_fromSlave;
}
