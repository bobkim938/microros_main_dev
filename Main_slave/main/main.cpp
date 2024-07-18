#include "DI_DO_SPI.h"
#include <iostream>
#include "i2c_slave.h"
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <chrono>

# define DO0 GPIO_NUM_35 // 1
# define DO1 GPIO_NUM_36 // 2
# define DO2 GPIO_NUM_37 // 4
# define DO3 GPIO_NUM_38 // 8
# define DO4 GPIO_NUM_3 // 16
# define DO5 GPIO_NUM_46 // 32
# define DO6 GPIO_NUM_39 // 64
# define DO7 GPIO_NUM_40 // 128
# define DO8 GPIO_NUM_41 // 256
# define DO9 GPIO_NUM_42 // 512

# define BMS GPIO_NUM_48
# define PASS1 GPIO_NUM_14
# define PASS2 GPIO_NUM_21

# define DI0 GPIO_NUM_2
# define DI1 GPIO_NUM_1
# define DI12 GPIO_NUM_20
# define DI13 GPIO_NUM_4
# define DI14 GPIO_NUM_5
# define DI15 GPIO_NUM_6
# define DI16 GPIO_NUM_7
# define DI18 GPIO_NUM_19
# define BATSW GPIO_NUM_47

#define GPIO_INPUT_PIN_SEL  ((1ULL<<DI0) | (1ULL<<DI1) | (1ULL<<BATSW))
#define ESP_INTR_FLAG_DEFAULT 0

static QueueHandle_t estop_evt_queue = NULL;
static QueueHandle_t batsw_evt_queue = NULL;

auto estop0_start = std::chrono::high_resolution_clock::now();
auto estop1_start = std::chrono::high_resolution_clock::now();
auto batSW_shutDown = std::chrono::high_resolution_clock::now();
bool ESTOP0_triggered = false;
bool ESTOP1_triggered = false;

bool batSW = false;
bool batSW_shutDown_flag = false;
uint8_t batSW_onCnt = 0;

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

static void IRAM_ATTR estop_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(estop_evt_queue, &gpio_num, NULL);
}

static void IRAM_ATTR batsw_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(batsw_evt_queue, &gpio_num, NULL);
}


static void estop_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(estop_evt_queue, &io_num, portMAX_DELAY)) {
            // printf("GPIO[%"PRIu32"] intr, val: %d\n", io_num, gpio_get_level((gpio_num_t)io_num));
			if (io_num == DI0) {
				estop0_start = std::chrono::high_resolution_clock::now();
				ESTOP0_triggered = false;
			}
			if (io_num == DI1) {
				estop1_start = std::chrono::high_resolution_clock::now();
				ESTOP1_triggered = false;
			}
			if (io_num == BATSW) {
				if(!batSW) {
					batSW = true;
				}
				else if(batSW) {
					batSW_shutDown = std::chrono::high_resolution_clock::now();
					batSW_shutDown_flag = true;
				}
			}
        }
    }
}

static void batsw_task(void* arg)
{
    uint32_t io_num;
    for (;;) {
        if (xQueueReceive(batsw_evt_queue, &io_num, portMAX_DELAY)) {

			if (io_num == BATSW) {
				if(!batSW) {
					batSW = true;
				}
				else if(batSW) {
					batSW_shutDown = std::chrono::high_resolution_clock::now();
					batSW_shutDown_flag = true;
				}
			}
		}
    }
}

bool ESPdi[8] = {};
uint8_t current_DI[3] = {}; // index 0 (MSB) -> index 2 (LSB)

void set_dOut(uint16_t dOut);
void set_bms(uint16_t dOut);
void read_DI();
void remap_DI(uint8_t di0_data, uint8_t di1_data);
void reset_gpio();


extern "C" void app_main(void) {
	reset_gpio();

	gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE;
    gpio_config(&io_conf);

    estop_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	batsw_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    xTaskCreate(estop_task, "ESTOP TRIGGER", 2048, NULL, 10, NULL);
	xTaskCreate(batsw_task, "BATSW TRIGGER", 2048, NULL, 10, NULL);

    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(DI0, estop_isr_handler, (void*) DI0);
    gpio_isr_handler_add(DI1, estop_isr_handler, (void*) DI1);
	gpio_isr_handler_add(BATSW, batsw_isr_handler, (void*) BATSW);
	
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
		if(i2c.get_bms() == true) {
			set_bms(dOut);
		}
		if(i2c.get_batSW() == true) {
			bool batSW = gpio_get_level(BATSW);
			i2c.set_batSW(batSW);
		}

		auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_time_estop0 = std::chrono::duration_cast<std::chrono::milliseconds>(now - estop0_start).count();
		auto elapsed_time_estop1 = std::chrono::duration_cast<std::chrono::milliseconds>(now - estop1_start).count();

		if(batSW_shutDown_flag && gpio_get_level(BATSW) == 1) {
			auto elapsed_time_batSW = std::chrono::duration_cast<std::chrono::milliseconds>(now - batSW_shutDown).count();
			if(elapsed_time_batSW > 2000) {
				batSW = false;
				batSW_shutDown_flag = false;
			}
		}
        
        if (elapsed_time_estop0 > 110) {
			printf("ESTOP0 TRIGGERED\n");
            ESTOP0_triggered = true;
        }
		if (elapsed_time_estop1 > 110) {
			printf("ESTOP1 TRIGGERED\n");
			ESTOP1_triggered = true;
		}

		if(ESTOP0_triggered && ESTOP1_triggered) {
			printf("EMERGENCY STOP\n");
		}
		if(batSW) {
			printf("BATSW ON\n");
		}
		else if(!batSW) {
			printf("BATSW OFF\n");
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

void set_bms(uint16_t dOut) {
	bool bms = dOut & 0x0001; // bit 0 indicating BMS
	bool pass1 = dOut & 0x0002; // bit 1 indicating PASS1
	bool pass2 = dOut & 0x0004; // bit 2 indicating PASS2

	gpio_set_level(BMS, bms);
	gpio_set_level(PASS1, pass1);
	gpio_set_level(PASS2, pass2);
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
	gpio_reset_pin(BMS);
	gpio_reset_pin(PASS1);
	gpio_reset_pin(PASS2);

	// gpio_reset_pin(DI0);
	// gpio_reset_pin(DI1);
	// gpio_reset_pin(DI12);
	// gpio_reset_pin(DI13);
	// gpio_reset_pin(DI14);
	// gpio_reset_pin(DI15);
	// gpio_reset_pin(DI16);
	// gpio_reset_pin(DI18);
	// gpio_reset_pin(BATSW);

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
	gpio_set_direction(BMS, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS1, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS2, GPIO_MODE_OUTPUT);

	// gpio_set_direction(DI0, GPIO_MODE_INPUT);
	// gpio_set_direction(DI1, GPIO_MODE_INPUT);
	gpio_set_direction(DI12, GPIO_MODE_INPUT);
	gpio_set_direction(DI13, GPIO_MODE_INPUT);
	gpio_set_direction(DI14, GPIO_MODE_INPUT);
	gpio_set_direction(DI15, GPIO_MODE_INPUT);
	gpio_set_direction(DI16, GPIO_MODE_INPUT);
	gpio_set_direction(DI18, GPIO_MODE_INPUT);
	// gpio_set_direction(BATSW, GPIO_MODE_INPUT);
}
