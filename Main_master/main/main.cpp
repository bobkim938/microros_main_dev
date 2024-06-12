#include <string.h>
#include <stdio.h>
#include <unistd.h>

#include "i2c_master.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/uart.h"
#include "DI_DO_SPI.h"

i2c_master_config i2c_conf = {
	.sda = GPIO_NUM_18,
	.scl = GPIO_NUM_19
};

DI_DO_SPI_config DD_spi_config = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_10,
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