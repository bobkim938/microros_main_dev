#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include <cstdint>

#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                      

static const char *TAG = "i2c-master";

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint8_t slaveAddr;
} i2c_master_config;

// DO 20 bits
// DI 24 bits

class i2c_master {
    public:
        i2c_master(i2c_master_config* conf);
        esp_err_t begin();
        esp_err_t i2c_send_DO(uint8_t* data);
        esp_err_t i2c_read_DI();
    
    private:
        i2c_master_bus_config_t i2c_mst_config = {};
        i2c_master_bus_handle_t i2c_mst_handle;
        i2c_device_config_t ESP_slave = {};
        i2c_master_dev_handle_t i2c_master_handle;
        esp_err_t ret;

        uint8_t DI_data[3];
        uint8_t recvbuf[0] = {};

};
