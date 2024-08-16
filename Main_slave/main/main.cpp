//#include <iostream>
//#include <stdlib.h>
//#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp_err.h"

//#include <chrono>
//#include "dido_spi.h"
//#include "shoalbot_slave_i2c.h"
//#include "dmx_485.h"
//#include "estop_pcnt.h"

#define JETSON_24 GPIO_NUM_0
#define DI_1 GPIO_NUM_1
#define DI_0 GPIO_NUM_2
#define DO_4 GPIO_NUM_3	
#define DI_13 GPIO_NUM_4
#define DI_14 GPIO_NUM_5
#define DI_15 GPIO_NUM_6
#define DI_16 GPIO_NUM_7
#define RS3_DE GPIO_NUM_8
#define SPI_CS_1 GPIO_NUM_9 // si8380 (DI10,11,17,19,20-23)
#define SPI_CS_2 GPIO_NUM_10 // si8380 (DI2-9)
#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define PASS_1 GPIO_NUM_14
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_16
#define RS3_TX GPIO_NUM_17
#define RS3_RX GPIO_NUM_18
#define DI_18 GPIO_NUM_19
#define DI_12 GPIO_NUM_20
#define PASS_2 GPIO_NUM_21
#define DO_0 GPIO_NUM_35	// 1
#define DO_1 GPIO_NUM_36	// 2
#define DO_2 GPIO_NUM_37	// 4
#define DO_3 GPIO_NUM_38	// 8
#define DO_6 GPIO_NUM_39	// 64
#define DO_7 GPIO_NUM_40	// 128
#define DO_8 GPIO_NUM_41	// 256
#define DO_9 GPIO_NUM_42	// 512
#define DO_5 GPIO_NUM_46	// 32
#define BOOTKEY GPIO_NUM_47	
#define BMS GPIO_NUM_48

static const char *ESTOP_PCNT_TAG = "estop_pcnt";
static const char *DIDO_SPI_TAG = "dido_spi";
int pulse_count = 0, prev_pulse_count;
int8_t shoalbot_amr_state = 5;
/* shoalbot_amr_state I will decide the state based on priority, then tell DMX
1: ESTOP
2: ERROR
3: LOWBAT
4: CHRGNG
5: IDLE
6: SHWBAT
11: BLOCK
12: LEFT
13: RIGHT
14: MOVE
*/

int32_t registry_di, registry_do;
int8_t registry_amr_state, registry_bms_level;
uint8_t BMS_percentage = 0;
uint16_t current_NavState = 0;

uint16_t dOut = 0x0000;
bool new_DO = false;

pcnt_unit_config_t unit_config = {
	.low_limit = -100,
    .high_limit = 100,
};
pcnt_unit_handle_t pcnt_unit = NULL;
pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 1000,
};
pcnt_chan_config_t chan_a_config = {
    .edge_gpio_num = DI_0,
};
pcnt_channel_handle_t pcnt_chan_a = NULL;
pcnt_chan_config_t chan_b_config = {
    .edge_gpio_num = DI_1,
};
pcnt_channel_handle_t pcnt_chan_b = NULL;

spi_device_handle_t handle;
spi_bus_config_t bus_config = {
	.mosi_io_num = SPI_MOSI,
	.data0_io_num = -1,
	.miso_io_num = SPI_MISO,
	.data1_io_num = -1,
	.sclk_io_num = SPI_SCLK,
	.quadwp_io_num = -1,
	.data2_io_num = -1,
	.quadhd_io_num = -1,
	.data3_io_num = -1,
	.data4_io_num = -1,
	.data5_io_num = -1,
	.data6_io_num = -1,
	.data7_io_num = -1,
	
};
// spi_device_interface_config_t DiDo_cfg = {
// 	.command_bits = 0,
// 	.address_bits = 0,
// 	.dummy_bits = 0,
// 	.mode = 2,
// 	.clock_source = SPI_CLK_SRC_DEFAULT,
// 	.duty_cycle_pos = 128,
// 	.cs_ena_pretrans = 0,
// 	.cs_ena_posttrans = 0,
// 	.clock_speed_hz = 1000000, // 1 Mhz
// 	.input_delay_ns = 0,
// 	.spics_io_num = SPI_SCLK,
// 	.flags = 
// 	.queue_size = 5,
// 	.pre_cb = ,
// 	.post_cb = ,
// };
// spi_transaction_t t;


// i2c_slave_config i2c_conf = {
// 	.sda = I2C_SDA, 
// 	.scl = I2C_SCL, 
// 	.slaveAddr = 0x0A
// };
//shoalbot_slave_i2c my_i2c(&i2c_conf);
// DI_DO_SPI_config DD_spi_config_0 = {
// 	.miso = SPI_MISO, 
// 	.mosi = SPI_MOSI, 
// 	.sclk = SPI_SCLK, 
// 	.cs = SPI_CS_2, 
// 	.bus_init = false,
// };
//DI_DO_SPI di0(&DD_spi_config_0);
// DI_DO_SPI_config DD_spi_config_1 = {
// 	.miso = SPI_MISO, 
// 	.mosi = SPI_MOSI, 
// 	.sclk = SPI_SCLK, 
// 	.cs = SPI_CS_1, 
// 	.bus_init = true,
// };
//DI_DO_SPI di1(&DD_spi_config_1);

//DMX::AMRState amr_state = DMX::AMRState::AMR_IDLE;

bool ESPdi[8] = {};
uint8_t current_state[6] = {}; // index 0 (MSB) -> index 2 (LSB)
uint8_t di0_data, di1_data;

void set_dOut(void* arg) {
	while(1) {
		if(new_DO) {
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
			gpio_set_level(DO_0, d0);
			gpio_set_level(DO_1, d1);
			gpio_set_level(DO_2, d2);
			gpio_set_level(DO_3, d3);
			gpio_set_level(DO_4, d4);
			gpio_set_level(DO_5, d5);
			gpio_set_level(DO_6, d6);
			gpio_set_level(DO_7, d7);
			gpio_set_level(DO_8, d8);
			gpio_set_level(DO_9, d9);
			new_DO = false;
		}
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	vTaskDelete(NULL);
}

void readMap_DO_DI(void* arg) {
	while(1) {
		// FOR DI
		ESPdi[0] = gpio_get_level(DI_0);
		ESPdi[1] = gpio_get_level(DI_1);
		ESPdi[2] = gpio_get_level(DI_12);
		ESPdi[3] = gpio_get_level(DI_13);
		ESPdi[4] = gpio_get_level(DI_14);
		ESPdi[5] = gpio_get_level(DI_15);
		ESPdi[6] = gpio_get_level(DI_16);
		ESPdi[7] = gpio_get_level(DI_18);
		// di0_data = di0.read_di(); // printf("spi0: 0x%02X\n", di0_data);
		// di1_data = di1.read_di(); // printf("spi1: 0x%02X\n", di1_data);

		// FOR DO
		bool r_d0 = gpio_get_level(DO_0);
		bool r_d1 = gpio_get_level(DO_1);
		bool r_d2 = gpio_get_level(DO_2);
		bool r_d3 = gpio_get_level(DO_3);
		bool r_d4 = gpio_get_level(DO_4);
		bool r_d5 = gpio_get_level(DO_5);
		bool r_d6 = gpio_get_level(DO_6);
		bool r_d7 = gpio_get_level(DO_7);
		bool r_d8 = gpio_get_level(DO_8);
		bool r_d9 = gpio_get_level(DO_9);
		
		// di0_data -> 0b{di9, di8, di7, di6, di5, di4, di3, di2}
		// di1_data -> 0b{di23, di22, di21, di20, di19, di17, di11, di10}
		// etract di17 only from di1_data
		// ESPdi -> 0b{di0, di1, di12, di13, di14, di15, di16, di18}
		// remap di0_data, di1_data, di_esp to current_state

		// current_state[5] -> 0b{amr_state[7], amr_state[6], amr_state[5], amr_state[4], amr_state[3], amr_state[2], amr_state[1], amr_state[0]}
		// current_state[4] -> 0b{0, 0, 0, 0, amr_state[9], amr_state[8], do9, do8}
		// current_state[3] -> 0b{do1, do0, do2, do3, do4, do5, do6, do7}
		// current_state[2] -> 0b{di7, di6, di5, di4, di3, di2, di1, di0}
		// current_state[1] -> 0b{di15, di14, di13, di12, di11, di10, di9, di8}
		// current_state[0] -> 0b{di23, di22, di21, di20, di19, di18, di17, di16}

		current_state[5] = 0xFF;
		current_state[4] = 0x00 | r_d8 | (r_d9 << 1) | 0b10 << 2;
		current_state[3] = 0x00 | r_d0 | (r_d1 << 1) | (r_d2 << 2) | (r_d3 << 3) | (r_d4 << 4) | (r_d5 << 5) | (r_d6 << 6) | (r_d7 << 7);
		current_state[2] = 0x00 | ESPdi[0] | (ESPdi[1] << 1);
		current_state[2] |= (di0_data << 2);
		current_state[1] = 0x00 | (ESPdi[2] << 4) | (ESPdi[3] << 5) | (ESPdi[4] << 6) | (ESPdi[5] << 7);
		current_state[1] |= (di0_data >> 6);
		current_state[0] = 0x00 | (ESPdi[6]) | (ESPdi[7] << 2);
		current_state[0] |= ((di1_data & 0x04) >> 1) | (di1_data & 0xF8);
	/*	printf("DI0: 0x%02X\n", current_DI[0]);
		printf("DI1: 0x%02X\n", current_DI[1]);
		printf("DI2: 0x%02X\n", current_DI[2]);*/
		vTaskDelay(pdMS_TO_TICKS(1));
	}
	vTaskDelete(NULL);
}

/*
void rs485_task(void *arg) { // DMX task
	while (1) {
		DMX::SetAMRState(DMX::AMRState::AMR_ERROR, 33);;  // Update LED color based on amr_state.
		vTaskDelay(pdMS_TO_TICKS(2000));
	}
	vTaskDelete(NULL);
}
*/
/*
void i2c_task(void* arg) {
	while(1) {
		my_i2c.i2c_read();
		if(my_i2c.get_state() == true) {
			my_i2c.set_state(current_state);
		}
		if(my_i2c.get_do() == true) {
			dOut = my_i2c.return_DO();
			new_DO = true;
			current_NavState = my_i2c.return_NavState();
			BMS_percentage = my_i2c.return_BMS();
			ESP_LOGI("I2C", "current DO State: 0x%04X\n",dOut);
			ESP_LOGI("I2C", "current Nav State: 0x%04X\n",current_NavState);
			ESP_LOGI("I2C", "Battery Percentage: %u\n", BMS_percentage);
		}
	}
	vTaskDelete(NULL);
}
*/
void check_estop(void* arg) {
	while(1) {
		ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        ESP_LOGI(ESTOP_PCNT_TAG, "Pulse count: %d", pulse_count);
		if ((pulse_count - prev_pulse_count) < 2) { // no new estop pulse is detected for the half cycle
			ESP_LOGI(ESTOP_PCNT_TAG, "ESTOP !");
			shoalbot_amr_state = 1;
			gpio_set_level(PASS_1, 0); 
			gpio_set_level(PASS_2, 0); 
		}
		else {
			shoalbot_amr_state = 5;
			gpio_set_level(PASS_1, 1); 
			gpio_set_level(PASS_2, 1); 
		}
		prev_pulse_count = pulse_count;
		if (pulse_count>97) { // a high enough odd number
			ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
			prev_pulse_count = 0;
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
	vTaskDelete(NULL);
}




extern "C" void app_main(void) {

	gpio_reset_pin(DO_0);
	gpio_reset_pin(DO_1);
	gpio_reset_pin(DO_2);
	gpio_reset_pin(DO_3);
	gpio_reset_pin(DO_4);
	gpio_reset_pin(DO_5);
	gpio_reset_pin(DO_6);
	gpio_reset_pin(DO_7);
	gpio_reset_pin(DO_8);
	gpio_reset_pin(DO_9);
	gpio_reset_pin(BMS);
	gpio_reset_pin(PASS_1);
	gpio_reset_pin(PASS_2);
	gpio_reset_pin(DI_12);
	gpio_reset_pin(DI_13);
	gpio_reset_pin(DI_14);
	gpio_reset_pin(DI_15);
	gpio_reset_pin(DI_16);
	gpio_reset_pin(DI_18);
	gpio_reset_pin(BOOTKEY);
	gpio_set_direction(DO_0, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_3, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_4, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_5, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_6, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_7, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_8, GPIO_MODE_OUTPUT);
	gpio_set_direction(DO_9, GPIO_MODE_OUTPUT);
	gpio_set_direction(BMS, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS_1, GPIO_MODE_OUTPUT);
	gpio_set_direction(PASS_2, GPIO_MODE_OUTPUT);
	gpio_set_direction(DI_12, GPIO_MODE_INPUT);
	gpio_set_direction(DI_13, GPIO_MODE_INPUT);
	gpio_set_direction(DI_14, GPIO_MODE_INPUT);
	gpio_set_direction(DI_15, GPIO_MODE_INPUT);
	gpio_set_direction(DI_16, GPIO_MODE_INPUT);
	gpio_set_direction(DI_18, GPIO_MODE_INPUT);
	gpio_set_direction(BOOTKEY, GPIO_MODE_INPUT);
	gpio_set_pull_mode(BOOTKEY, GPIO_PULLUP_ONLY);

	gpio_set_level(BMS, 1);
	gpio_set_level(PASS_1, 1); // Kinco enable 
	gpio_set_level(PASS_2, 1); // N.C.
	vTaskDelay(pdMS_TO_TICKS(100));
	gpio_set_level(DO_0, 1); // Lidar 24V
	gpio_set_level(DO_1, 1); // LED	24V
	gpio_set_level(DO_2, 0); // Relay K2 to close Auto Charging's 48V loop
	gpio_set_level(DO_3, 1); // Relay K3 for Kinco Power

    ESP_LOGI(ESTOP_PCNT_TAG, "Install pcnt unit");
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));
	ESP_LOGI(ESTOP_PCNT_TAG, "Set glitch filter");
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));
	ESP_LOGI(ESTOP_PCNT_TAG, "Install pcnt channels");
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));
	ESP_LOGI(ESTOP_PCNT_TAG, "Set edge and level actions for pcnt channels");
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
	ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_LOGI(ESTOP_PCNT_TAG, "Enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(ESTOP_PCNT_TAG, "Clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(ESTOP_PCNT_TAG, "Start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

	ESP_LOGI(DIDO_SPI_TAG, "Initialize spi bus");
	ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));	


//	di0.begin();
//	di1.begin();
//	DMX::Initialize(DMXDirection::DMX_DIR_OUTPUT, 1, 512);
//	xTaskCreate(rs485_task, "rs485_task", 16000, NULL, 1, NULL);
//	xTaskCreate(readMap_DO_DI, "readMap_state", 16000, NULL, 5, NULL);
//	xTaskCreate(i2c_task, "i2c_task", 16000, NULL, 5, NULL);
//	xTaskCreatePinnedToCore(set_dOut, "set_dOut", 16000, NULL, 5, NULL, 1);
	xTaskCreate(check_estop, "check_estop", 16000, NULL, 5, NULL);

// 	while(1) {
		// ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        // ESP_LOGI(ESTOP_PCNT_TAG, "Pulse count: %d", pulse_count);

		// if ((pulse_count - prev_pulse_count) < 2) { // no new estop pulse is detected for the half cycle
		// 	ESP_LOGI(ESTOP_PCNT_TAG, "ESTOP !");
		// 	shoalbot_amr_state = 1;
		// 	gpio_set_level(PASS_1, 0); 
		// 	gpio_set_level(PASS_2, 0); 
		// }
		// else {
		// 	shoalbot_amr_state = 5;
		// 	gpio_set_level(PASS_1, 1); 
		// 	gpio_set_level(PASS_2, 1); 
		// }
		// prev_pulse_count = pulse_count;
		// if (pulse_count>97) { // a high enough odd number
		// 	ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
		// 	prev_pulse_count = 0;
		// }
		// vTaskDelay(pdMS_TO_TICKS(50));
 	//}
}