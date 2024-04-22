#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include <cstring>
#include "esp_log.h"

typedef struct {
    int sda;
    int scl;
    uint16_t slaveAddr;
} i2c_slave_config;

static const char *TAG = "i2c-slave";

class i2c_slave {
    public:
        i2c_slave(i2c_slave_config* slave_config);
        esp_err_t begin();
        uint8_t slave_read_buffer();

    private:
        i2c_config_t conf_slave = {};
        i2c_port_t i2c_port = I2C_NUM_0;          
        int I2C_SLAVE_RX_BUF_LEN = 255;   
        int I2C_SLAVE_TX_BUF_LEN = 255;
        uint8_t pong[255] = {};         
};