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

# define DI0 GPIO_NUM_2
# define DI1 GPIO_NUM_1
# define DI12 GPIO_NUM_20
# define DI13 GPIO_NUM_4
# define DI14 GPIO_NUM_5
# define DI15 GPIO_NUM_6
# define DI16 GPIO_NUM_7
# define DI18 GPIO_NUM_19

i2c_slave_config i2c_conf = {
    .sda = GPIO_NUM_15, 
    .scl = GPIO_NUM_16, 
	.slaveAddr = 0x0A
};

DI_DO_SPI_config DD_spi_config_0 = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_10,
	.bus_init = false,
};

DI_DO_SPI_config DD_spi_config_1 = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_9,
	.bus_init = true,
};

bool ESPdi[8] = {};
uint8_t current_DI[3] = {}; // index 0 (MSB) -> index 2 (LSB)

void set_dOut(uint16_t dOut);
void read_DI();
void remap_DI(uint8_t di0_data, uint8_t di1_data);
void reset_gpio();

extern "C" void app_main(void) {
	reset_gpio();

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

	gpio_set_direction(DI0, GPIO_MODE_INPUT);
	gpio_set_direction(DI1, GPIO_MODE_INPUT);
	gpio_set_direction(DI12, GPIO_MODE_INPUT);
	gpio_set_direction(DI13, GPIO_MODE_INPUT);
	gpio_set_direction(DI14, GPIO_MODE_INPUT);
	gpio_set_direction(DI15, GPIO_MODE_INPUT);
	gpio_set_direction(DI16, GPIO_MODE_INPUT);
	gpio_set_direction(DI18, GPIO_MODE_INPUT);

	// gpio_reset_pin(GPIO_NUM_48);
	// gpio_reset_pin(GPIO_NUM_21);
	// gpio_set_direction(GPIO_NUM_48, GPIO_MODE_OUTPUT);
	// gpio_set_direction(GPIO_NUM_21, GPIO_MODE_OUTPUT);
	// while(1) {
	// 	gpio_set_level(GPIO_NUM_48, 1);
	// 	gpio_set_level(GPIO_NUM_21, 1);
	// 	vTaskDelay(1000 / portTICK_PERIOD_MS);
	// 	gpio_set_level(GPIO_NUM_48, 0);
	// 	gpio_set_level(GPIO_NUM_21, 0);
	// 	vTaskDelay(1000 / portTICK_PERIOD_MS);
	// }
	// gpio_set_level(GPIO_NUM_48, 1);
	// gpio_set_level(GPIO_NUM_21, 1);

	i2c_slave i2c(&i2c_conf);
	DI_DO_SPI di0(&DD_spi_config_0);
	DI_DO_SPI di1(&DD_spi_config_1);
	di0.begin();
	di1.begin();
	while(1) {
		uint16_t dOut = i2c.i2c_read();
		if(i2c.get_di() == true) {
			uint8_t di0_data = di0.read_di();
			printf("spi0: 0x%02X\n", di0_data);
			uint8_t di1_data = di1.read_di();
			printf("spi1: 0x%02X\n", di1_data);
			read_DI();
			remap_DI(di0_data, di1_data);
			i2c.set_di(current_DI);
		}
		if(i2c.get_do() == true) {
			set_dOut(dOut);
			printf("DO: %d\n", dOut);
		}
	}
}

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
	printf("d0: %d\n", d0);
	printf("d1: %d\n", d1);
	printf("d2: %d\n", d2);
	printf("d3: %d\n", d3);
	printf("d4: %d\n", d4);
	printf("d5: %d\n", d5);
	printf("d6: %d\n", d6);
	printf("d7: %d\n", d7);
	printf("d8: %d\n", d8);
	printf("d9: %d\n", d9);
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

void read_DI() {
	ESPdi[0] = gpio_get_level(DI0);
	ESPdi[1] = gpio_get_level(DI1);
	ESPdi[2] = gpio_get_level(DI12);
	ESPdi[3] = gpio_get_level(DI13);
	ESPdi[4] = gpio_get_level(DI14);
	ESPdi[5] = gpio_get_level(DI15);
	ESPdi[6] = gpio_get_level(DI16);
	ESPdi[7] = gpio_get_level(DI18);

	printf("DI0: %d\n", ESPdi[0]);
	printf("DI1: %d\n", ESPdi[1]);
	printf("DI12: %d\n", ESPdi[2]);
	printf("DI13: %d\n", ESPdi[3]);
	printf("DI14: %d\n", ESPdi[4]);
	printf("DI15: %d\n", ESPdi[5]);
	printf("DI16: %d\n", ESPdi[6]);
	printf("DI18: %d\n", ESPdi[7]);
}

void remap_DI(uint8_t di0_data, uint8_t di1_data) {
	// di0_data -> 0b{di9, di8, di7, di6, di5, di4, di3, di2}
	// di1_data -> 0b{di23, di22, di21, di20, di19, di17, di11, di10}
	// etract di17 only from di1_data
	// ESPdi -> 0b{di0, di1, di12, di13, di14, di15, di16, di18}
	// remap di0_data, di1_data, di_esp to current_DI

	// current_DI[2] -> 0b{di7, di6, di5, di4, di3, di2, di1, di0}
	// current_DI[1] -> 0b{di15, di14, di13, di12, di11, di10, di9, di8}
	// current_DI[0] -> 0b{di23, di22, di21, di20, di19, di18, di17, di16}
	current_DI[2] = 0x00 | ESPdi[0] | (ESPdi[1] << 1);
	current_DI[2] |= (di0_data << 2);
	current_DI[1] = 0x00 | (ESPdi[2] << 4) | (ESPdi[3] << 5) | (ESPdi[4] << 6) | (ESPdi[5] << 7);
	current_DI[1] |= (di0_data >> 6);
	current_DI[0] = 0x00 | (ESPdi[6]) | (ESPdi[7] << 2);
	current_DI[0] |= ((di1_data & 0x04) >> 1) | (di1_data & 0xF8);
	printf("DI0: 0x%02X\n", current_DI[0]);
	printf("DI1: 0x%02X\n", current_DI[1]);
	printf("DI2: 0x%02X\n", current_DI[2]);
}

void reset_gpio() {
	gpio_reset_pin(DO0);
	gpio_reset_pin(DO1);
	gpio_reset_pin(DO2);
	gpio_reset_pin(DO3);
	gpio_reset_pin(DO4);
	gpio_reset_pin(DO5);
	gpio_reset_pin(DO6);
	gpio_reset_pin(DO7);
	gpio_reset_pin(DO8);
	gpio_reset_pin(DO9);

	gpio_reset_pin(DI0);
	gpio_reset_pin(DI1);
	gpio_reset_pin(DI12);
	gpio_reset_pin(DI13);
	gpio_reset_pin(DI14);
	gpio_reset_pin(DI15);
	gpio_reset_pin(DI16);
	gpio_reset_pin(DI18);
}