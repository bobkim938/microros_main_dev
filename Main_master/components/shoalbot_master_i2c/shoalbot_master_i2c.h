#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                      

static const char *TAG = "i2c-master";

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t slaveAddr;
} i2c_master_config;

typedef struct {
    i2c_master_bus_config_t i2c_mst_config;
    i2c_master_bus_handle_t i2c_mst_handle;
    i2c_device_config_t ESP_slave;
    i2c_master_dev_handle_t i2c_master_handle;
    esp_err_t ret;
    uint8_t state_data[6];
    uint8_t DO_slave[2];
    // uint8_t AMR_state[2];
    uint8_t AMR_state[1];
    uint8_t DI_cmd[1];
    uint8_t batSW_cmd[1];
    uint32_t DI_fromSlave;
    uint8_t diCnt;
} shoalbot_master_i2c;

void shoalbot_master_i2c_init(shoalbot_master_i2c* obj, i2c_master_config* conf);
esp_err_t shoalbot_master_i2c_begin(shoalbot_master_i2c* obj);
esp_err_t shoalbot_master_i2c_i2c_send_DO(shoalbot_master_i2c* obj, uint8_t* data);
uint32_t shoalbot_master_i2c_read_state(shoalbot_master_i2c* obj);
