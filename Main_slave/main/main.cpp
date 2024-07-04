#include "DI_DO_SPI.h"
#include <iostream>
#include "i2c_slave.h"

# define DO0 GPIO_NUM_35
# define DO1 GPIO_NUM_36
# define DO2 GPIO_NUM_37
# define DO3 GPIO_NUM_38
# define DO4 GPIO_NUM_3
# define DO5 GPIO_NUM_46
# define DO6 GPIO_NUM_39
# define DO7 GPIO_NUM_40
# define DO8 GPIO_NUM_41
# define DO9 GPIO_NUM_42

i2c_slave_config i2c_conf = {
    .sda = GPIO_NUM_15, 
    .scl = GPIO_NUM_16, 
	.slaveAddr = 0x0A
};

DI_DO_SPI_config DD_spi_config = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_10,
};

void set_dOut(uint16_t dOut) {
	bool d0 = dOut & 0x0001;
	bool d1 = dOut & 0x0002;
	bool d2 = dOut & 0x0004;
	bool d3 = dOut & 0x0008;
	bool d4 = dOut & 0x0010;
	bool d5 = dOut & 0x0020;
	bool d6 = dOut & 0x0040;
	bool d7 = dOut & 0x0080;
	bool d8 = dOut & 0x0100;
	bool d9 = dOut & 0x0200;
	gpio_set_level(DO0, d0);
	gpio_set_level(DO1, d1);
	gpio_set_level(DO2, d2);
	gpio_set_level(DO3, d3);
	gpio_set_level(DO4, d4);
	gpio_set_level(DO5, d5);
	gpio_set_level(DO6, d6);
	gpio_set_level(DO7, d7);
	gpio_set_level(DO8, d8);
	gpio_set_level(DO9, d9);
}

extern "C" void app_main(void) {
	gpio_set_direction(DO0, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO1, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO2, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO3, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO4, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO5, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO6, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO7, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO8, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO9, GPIO_MODE_OUTPUT);

	i2c_slave i2c(&i2c_conf);
	uint8_t current_DI[3] = {0x13, 0x23, 0x33}; // DI values read currently
	while(1) {
		uint16_t dOut = i2c.i2c_read();
		if(i2c.get_di() == true) {
			i2c.set_di(current_DI);
		}
		if(i2c.get_do() == true) {
			set_dOut(dOut);
			printf("DO: %d\n", dOut);
		}
	}
}