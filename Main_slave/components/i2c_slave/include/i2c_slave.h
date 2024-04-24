#include "driver/i2c_slave.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <cstring>
#include "esp_log.h"

typedef struct {
    gpio_num_t sda;
    gpio_num_t scl;
    uint16_t slaveAddr;
} i2c_slave_config;

static const char *TAG = "i2c-slave";

class i2c_slave {
    public:
        i2c_slave(i2c_slave_config* slave_config);


    private:
        i2c_slave_config_t slv_conf = {};
        i2c_slave_dev_handle_t slv_handle;
};