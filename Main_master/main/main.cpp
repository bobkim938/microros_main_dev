#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "i2c_master.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"

i2c_master_config i2c_conf = {
	.sda = GPIO_NUM_18,
	.scl = GPIO_NUM_19
};

extern "C" void app_main(void) {
	i2c_master i2c(&i2c_conf);
	i2c.add_slave_device(0x0A);
	uint8_t data = 1;
	while(1) {
		i2c.i2c_transmit(&data);
		data++;
		if(data > 10) {
			data = 1;
		}
		vTaskDelay(1000 / portTICK_PERIOD_MS);
	}
}