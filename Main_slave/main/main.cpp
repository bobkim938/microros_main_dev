#include "DI_DO_SPI.h"
#include <chrono>
#include <iostream>
#include "i2c_slave.h"

i2c_slave_config i2c_conf = {
	.sda = GPIO_NUM_18,
	.scl = GPIO_NUM_19,
	.slaveAddr = 0x0A
};

DI_DO_SPI_config DD_spi_config = {
    .miso = GPIO_NUM_13,
    .mosi = GPIO_NUM_11,
    .sclk = GPIO_NUM_12,
    .cs = GPIO_NUM_10,
};

extern "C" void app_main(void) {
	// i2c_master i2c(&i2c_conf);
	// i2c.add_slave_device(0x0A);
	// uint8_t data = 1;
	// while(1) {
	// 	i2c.i2c_transmit(&data);
	// 	data++;
	// 	if(data > 10) {
	// 		data = 1;
	// 	}
	// 	vTaskDelay(1000 / portTICK_PERIOD_MS);
	// }

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