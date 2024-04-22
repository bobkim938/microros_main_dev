#include <iostream>
#include "i2c_master.h"

uint8_t slave_cmd[15] = {};

i2c_master_config i2c_master_conf = {
    .sda = 18,
    .scl = 19,
    .slaveESP_addr = 0x0A
};

extern "C" void app_main(void)
{
    i2c_master i2c_master_obj(&i2c_master_conf);
    i2c_master_obj.begin();
    slave_cmd[5] = 1;
    uint8_t* data_rd = 0;

    while(1) {
        i2c_master_obj.i2c_master_send(slave_cmd, sizeof(slave_cmd));
        // esp_err_t ret = i2c_master_obj.i2c_master_read_slave(data_rd, 2);
        // if(ret == ESP_OK) {
        //     std::cout << "Data read: " << data_rd[0] << " " << data_rd[1] << std::endl;
        // }
        // else {
        //     std::cout << "Error reading data" << std::endl;
        // }
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
