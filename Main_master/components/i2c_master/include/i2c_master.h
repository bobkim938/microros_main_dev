#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                      

static const char *TAG = "i2c-master";

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
} i2c_master_config;

class i2c_master {
    public:
        i2c_master(i2c_master_config* conf);
        esp_err_t add_slave_device(uint16_t slave_addr);
        esp_err_t i2c_masterSendCMD(uint8_t* data_wr, size_t size);
    
    private:
        i2c_master_bus_config_t i2c_mst_config = {};
        i2c_master_bus_handle_t i2c_mst_handle;
        i2c_device_config_t ESP_slave = {};
        i2c_master_dev_handle_t i2c_slave_handle;
};

