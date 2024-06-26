#include "DI_DO_SPI.h"
#include <chrono>
#include <iostream>
#include "i2c_slave.h"

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

extern "C" void app_main(void) {
	i2c_slave i2c(&i2c_conf);
	uint8_t current_DI[3] = {0x13, 0x23, 0x33}; // DI values read currently
	while(1) {
		uint8_t a = i2c.i2c_read();
	}

	DI_DO_SPI dido(&DD_spi_config);
	dido.begin();
	std::chrono::microseconds total_duration(0);
	for(int i = 0; i < 100; i++) {
		auto start_time = std::chrono::high_resolution_clock::now();
		dido.test_read();
		auto end_time = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        total_duration += duration;
	}
	double average_duration = static_cast<double>(total_duration.count()) / 100;
    std::cout << "Average time per loop iteration: " << average_duration << " microseconds" << std::endl;
}