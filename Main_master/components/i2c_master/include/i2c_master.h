#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define ACK_CHECK_EN 0x1                        
#define ACK_CHECK_DIS 0x0                      

static const char *TAG = "i2c-master";

typedef struct {
    int sda;
    int scl;
    uint16_t slaveESP_addr;
} i2c_master_config;

class i2c_master { 
    public:
        i2c_master(i2c_master_config* master_config);
        esp_err_t begin();
        esp_err_t i2c_master_send(uint8_t* msg, size_t size);

    private:
        i2c_config_t master_config = {};
        i2c_port_t i2c_master_port = I2C_NUM_0;
        i2c_cmd_handle_t cmd;
        uint16_t slave_addr;
};
