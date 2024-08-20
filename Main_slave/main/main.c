//#include <iostream>
//#include <stdlib.h>
//#include <inttypes.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/pulse_cnt.h"
#include "driver/spi_master.h"
#include "driver/i2c_slave.h"
#include "driver/uart.h"
#include "driver/twai.h"
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
#define RS3_DE (GPIO_NUM_8)
#define SPI_CS_1 GPIO_NUM_9 // si8380 (DI10,11,17,19,20-23)
#define SPI_CS_2 GPIO_NUM_10 // si8380 (DI2-9)
#define SPI_MOSI GPIO_NUM_11
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13
#define PASS_1 GPIO_NUM_14
#define I2C_SDA GPIO_NUM_15
#define I2C_SCL GPIO_NUM_16
#define RS3_TX (GPIO_NUM_17)
#define RS3_RX (GPIO_NUM_18)
#define DI_18 GPIO_NUM_19
#define DI_12 GPIO_NUM_20
#define PASS_2 GPIO_NUM_21
#define DO_0 GPIO_NUM_35    // 1
#define DO_1 GPIO_NUM_36    // 2
#define DO_2 GPIO_NUM_37    // 4
#define DO_3 GPIO_NUM_38    // 8
#define DO_6 GPIO_NUM_39    // 64
#define DO_7 GPIO_NUM_40    // 128
#define DO_8 GPIO_NUM_41    // 256
#define DO_9 GPIO_NUM_42    // 512
#define DO_5 GPIO_NUM_46    // 32
#define BOOTKEY (GPIO_NUM_47)
#define BMS GPIO_NUM_48
 
static const char *BOOTKEY_TAG = "bootkey_gptimer";
static const char *ESTOP_PCNT_TAG = "estop_pcnt";
static const char *DIDO_SPI_TAG = "dido_spi";
static const char *I2C_SLAVE_TAG = "i2c_slave";
static const char *DMX_RS485_TAG = "dmx_rs485";
 
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
 
bool ESPdi[8] = {};
uint8_t current_state[6] = {}; // index 0 (MSB) -> index 2 (LSB)
uint8_t di0_data, di1_data;
uint16_t dOut = 0x0000;
bool new_DO = false;
 
/* ---------------FUNCTION PROTOTYPES---------------- */
void check_bootkey(void* arg);
void check_estop(void* arg);
uint8_t read_spi_1();
uint8_t read_spi_2();
void readMap_DO_DI(void* arg);
esp_err_t i2c_send_state(uint8_t* data, uint8_t index);
esp_err_t i2c_read();
void i2c_task(void *arg);
void createBuffer(bool TmpBufferCreate);
void dmx_write(uint16_t channel, uint8_t value);
void dmx_write_all(uint8_t * data, uint16_t start, size_t size);
void uart_send_task(void*pvParameters);
void set_led(const uint8_t color_cmd);
void set_dOut(void* arg);
 
 
/* -------------------- BOOTKEY -------------------- -------------------- --------------------
*/
void check_bootkey(void* arg) { //remember to disable this bootkey check during development/debug
    bool bootkey_pressed = false;
    uint32_t bootkey_start = 0; // start timer when bootkey is pressed
    while(1) {
        if (gpio_get_level(BOOTKEY)) {
            //ESP_LOGI(BOOTKEY_TAG, "BOOTKEY is pressed");
            if (!bootkey_pressed) {
                bootkey_pressed = true;
                bootkey_start = esp_log_timestamp();
            }
            else if (bootkey_pressed) {
                uint32_t elapsed_time = esp_log_timestamp() - bootkey_start;
                if (elapsed_time > 5000 ) { // 5 seconds
                    //ESP_LOGI(BOOTKEY_TAG, "BOOTKEY pressed for 5 seconds");
                    gpio_set_level(BMS, 0); // cut-off BMS
                }
            }
        }
        else {
            bootkey_pressed = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}
 
/* -------------------- ESTOP -------------------- -------------------- --------------------
*/
pcnt_unit_config_t unit_config = {
    .low_limit = -100,
    .high_limit = 100,
};
pcnt_unit_handle_t pcnt_unit = NULL;
pcnt_glitch_filter_config_t filter_config = {
    .max_glitch_ns = 10000,
};
pcnt_chan_config_t chan_a_config = {
    .edge_gpio_num = DI_0,
};
pcnt_channel_handle_t pcnt_chan_a = NULL;
pcnt_chan_config_t chan_b_config = {
    .edge_gpio_num = DI_1,
};
pcnt_channel_handle_t pcnt_chan_b = NULL;
int pulse_count = 0, prev_pulse_count;
bool estop_flag;

void check_estop(void* arg) {
    while(1) {
        ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
        // ESP_LOGI(ESTOP_PCNT_TAG, "Pulse count: %d", pulse_count);
        if ((pulse_count - prev_pulse_count) < 2) { // no new estop pulse is detected for the cycle
            // ESP_LOGI(ESTOP_PCNT_TAG, "ESTOP !");
            estop_flag = 1;
            gpio_set_level(PASS_1, 0);
            gpio_set_level(PASS_2, 0);
        }
        else {
            estop_flag = 0;
            gpio_set_level(PASS_1, 1);
            gpio_set_level(PASS_2, 1);
			
        }
        prev_pulse_count = pulse_count;
        if (pulse_count>95) { // a high enough odd number
            ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
            prev_pulse_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}
 
/* -------------------- DIDO -------------------- -------------------- --------------------
*/
spi_device_handle_t handle_1, handle_2;
spi_bus_config_t bus_config = {
    .mosi_io_num = SPI_MOSI,
    .miso_io_num = SPI_MISO,
    .sclk_io_num = SPI_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,    
};
spi_device_interface_config_t si8380s_1_cfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 2,
    .clock_source = SPI_CLK_SRC_DEFAULT,
    .duty_cycle_pos = 128,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = 1000000, // 1 Mhz
    .spics_io_num = SPI_CS_2,
    .queue_size = 5,
};
spi_device_interface_config_t si8380s_2_cfg = {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .mode = 2,
    .clock_source = SPI_CLK_SRC_DEFAULT,
    .duty_cycle_pos = 128,
    .cs_ena_pretrans = 0,
    .cs_ena_posttrans = 0,
    .clock_speed_hz = 1000000, // 1 Mhz
    .spics_io_num = SPI_CS_1,
    .queue_size = 5,
};
spi_transaction_t t_1, t_2;
uint8_t sendbuf_1[1] = {0x00}, sendbuf_2[1] = {0x00};
uint8_t recvbuf_1[1] = {0x00}, recvbuf_2[1] = {0x00};
uint8_t read_spi_1() {
    t_1.length = 8;
    t_1.tx_buffer = sendbuf_1;
    t_1.rx_buffer = recvbuf_1;
    sendbuf_1[0] |= (0b1 << 6);
    ESP_ERROR_CHECK(spi_device_transmit(handle_1, &t_1));
    sendbuf_1[0] = 0x00; // CHAN_STATUS
    ESP_ERROR_CHECK(spi_device_transmit(handle_1, &t_1));
    sendbuf_1[0] = 0x00;
    ESP_ERROR_CHECK(spi_device_transmit(handle_1, &t_1));
    return recvbuf_1[0];
}
 
uint8_t read_spi_2() {
    t_2.length = 8;
    t_2.tx_buffer = sendbuf_2;
    t_2.rx_buffer = recvbuf_2;
    sendbuf_2[0] |= (0b1 << 6);
    ESP_ERROR_CHECK(spi_device_transmit(handle_2, &t_2));
    sendbuf_2[0] = 0x00; // CHAN_STATUS
    ESP_ERROR_CHECK(spi_device_transmit(handle_2, &t_2));
    sendbuf_2[0] = 0x00;
    ESP_ERROR_CHECK(spi_device_transmit(handle_2, &t_2));
    return recvbuf_2[0];
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
        di0_data = read_spi_1(); // ESP_LOGI(DIDO_SPI_TAG, "SPI0: 0x%02X", di0_data);
        di1_data = read_spi_2(); // ESP_LOGI(DIDO_SPI_TAG, "SPI1: 0x%02X", di1_data);
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
        // current_state[3] -> 0b{do0, do1, do2, do3, do4, do5, do6, do7}
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
        //ESP_LOGI(DIDO_SPI_TAG, "DI16 to DI23: 0x%02X\n", current_state[0]); // printf("DI0: 0x%02X\n", current_DI[0]);
        //ESP_LOGI(DIDO_SPI_TAG, "DI8 to DI15: 0x%02X\n", current_state[1]); // printf("DI1: 0x%02X\n", current_DI[1]);
        //ESP_LOGI(DIDO_SPI_TAG, "DI0 to DI7: 0x%02X\n", current_state[2]); // printf("DI2: 0x%02X\n", current_DI[2]);
        //ESP_LOGI(DIDO_SPI_TAG, "DO0 to DO7: 0x%02X\n", current_state[3]);
        //ESP_LOGI(DIDO_SPI_TAG, "DO8, DO9: 0x%02X\n", current_state[4] & 0x03);
        //ESP_LOGI(DIDO_SPI_TAG, "amr_state 8,9: 0x%02X\n", current_state[4] & 0x0C);
        //ESP_LOGI(DIDO_SPI_TAG, "amr_state 0 to 7: 0x%02X\n", current_state[5]);
 
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    vTaskDelete(NULL);
}
 
/* -------------------- I2C -------------------- -------------------- --------------------
*/
static IRAM_ATTR bool i2c_slave_rx_done_callback(i2c_slave_dev_handle_t channel, const i2c_slave_rx_done_event_data_t *edata, void *user_data) {
    BaseType_t high_task_wakeup = pdFALSE;
    QueueHandle_t receive_queue = (QueueHandle_t)user_data;
    xQueueSendFromISR(receive_queue, edata, &high_task_wakeup);
    return high_task_wakeup == pdTRUE;
}
i2c_slave_config_t slv_conf = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = I2C_SDA,
    .scl_io_num = I2C_SCL,
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .send_buf_depth = 256,
    .slave_addr = 0x0A,
    .addr_bit_len = I2C_ADDR_BIT_LEN_7,
};
i2c_slave_dev_handle_t slv_handle;
i2c_slave_event_callbacks_t cbs = {
    .on_recv_done = i2c_slave_rx_done_callback,
};
i2c_slave_rx_done_event_data_t rx_data;
uint8_t I2C_DO_cnt = 0;
uint16_t I2C_parsed_data_DO = 0;
uint16_t I2C_parsed_data_NavState = 0;
uint8_t I2C_parsed_data_BMS = 0;
bool I2C_DO_ack_flag = false;
bool I2C_State_ack_flag = false;
bool I2C_new_State = false;
bool I2C_new_DO = false;
 
esp_err_t i2c_send_state(uint8_t* data, uint8_t index) {
    esp_err_t ret = ESP_OK;
    for(int i = 0; i < index; i++) {
        ret = i2c_slave_transmit(slv_handle, data + i, 1, -1);
        // printf("DI sent: %d\n", *(data + i));
        if(ret != ESP_OK) {
                //ESP_LOGE(TAG, "Failed to transmit data");
                return ret;
            }
    }
    // printf("DI sent\n");
    return ret;
}
 
esp_err_t i2c_read() {
    uint8_t* data_rcv = (uint8_t *)(malloc(sizeof(uint8_t)));
    if (data_rcv == NULL) {
        ESP_LOGE("I2C", "Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    *data_rcv = 0;
    QueueHandle_t receive_queue = xQueueCreate(1, sizeof(i2c_slave_rx_done_event_data_t));
    if (receive_queue == NULL) {
        free(data_rcv);
        ESP_LOGE("I2C", "Failed to create queue");
        return 0;
    }
    esp_err_t err = i2c_slave_register_event_callbacks(slv_handle, &cbs, receive_queue);
    if (err != ESP_OK) {
        free(data_rcv);
        vQueueDelete(receive_queue);
        return err;
    }
 
    err = i2c_slave_receive(slv_handle, data_rcv, 10);
    if (err != ESP_OK) {
        free(data_rcv);
        vQueueDelete(receive_queue);
        return err;
    }
    if(I2C_DO_cnt == 0) {
        I2C_parsed_data_DO = 0;
        I2C_parsed_data_NavState = 0;
    }
    if (xQueueReceive(receive_queue, &rx_data, pdMS_TO_TICKS(2000)) == pdPASS) {
        // if(I2C_DO_ack_flag && I2C_DO_cnt == 0) {
        //     // ESP_LOGI("I2C", "Data received 1: %d", *data_rcv);
        //     I2C_parsed_data_DO |= (*data_rcv << 8);
        //     I2C_DO_cnt++;
        // }
        if(I2C_DO_ack_flag && I2C_DO_cnt == 0) {
            // ESP_LOGI("I2C", "Data received 2: %d", *data_rcv);
            I2C_parsed_data_DO |= (*data_rcv);
            I2C_DO_cnt++;
        }
        else if(I2C_DO_ack_flag && I2C_DO_cnt == 1) {
            // ESP_LOGI("I2C", "Data received 3: %d", *data_rcv);
            I2C_parsed_data_NavState |= (*data_rcv << 8);
            I2C_DO_cnt++;
        }
        else if(I2C_DO_ack_flag && I2C_DO_cnt == 2) {
            // ESP_LOGI("I2C", "Data received 4: %d", *data_rcv);
            I2C_parsed_data_NavState |= *data_rcv;
            I2C_DO_cnt++;
        }
        else if(I2C_DO_ack_flag && I2C_DO_cnt == 3) {
            I2C_parsed_data_BMS = (*data_rcv);
            I2C_DO_cnt = 0;
            I2C_DO_ack_flag = false;
            I2C_new_DO = true;
        }
        else if(I2C_State_ack_flag) {
            i2c_send_state(current_state, 6);
            I2C_State_ack_flag = false;
        }
 
        if(*data_rcv == 0xBB && !I2C_DO_ack_flag && !I2C_State_ack_flag) {
            I2C_DO_ack_flag = true;
            // printf("DO acknowlodged\n");
        }
        else if(*data_rcv == 0xFF && !I2C_State_ack_flag && !I2C_DO_ack_flag) {
            I2C_State_ack_flag = true;
            I2C_new_State = true;
            // printf("DI acknowlodged\n");
        }
    } else {
        // ESP_LOGE("I2C", "Failed to receive data");
    }
    free(data_rcv); // free allocated memory
    vQueueDelete(receive_queue); // delete the queue to free resources
    return ESP_OK;
}
 
void i2c_task(void *arg) {
    while(1) {
        i2c_read();
        if(I2C_new_DO) {
            dOut = I2C_parsed_data_DO;
            // ESP_LOGI("I2C", "DO: 0x%04X\n", I2C_parsed_data_DO);
            // ESP_LOGI("I2C", "NavState: %d", I2C_parsed_data_NavState);
            // ESP_LOGI("I2C", "BMS: %d", I2C_parsed_data_BMS);
            if(I2C_parsed_data_NavState == 1) {
                set_led(1); // red
            }
            else if(I2C_parsed_data_NavState == 2) {
                set_led(2); // yellow
            }
            else if(I2C_parsed_data_NavState == 3) {
                set_led(3); // blue
            }
            else if(I2C_parsed_data_NavState == 4) {
                set_led(4); // green
            }
            else if(I2C_parsed_data_NavState == 5) {
                set_led(5); // mix
            }
            new_DO = true;
            I2C_new_DO = false;
        }
    }
    vTaskDelete(NULL);
}
 
 
 
/* -------------------- DMX -------------------- -------------------- --------------------
*/
#define DMX_UART_NUM (UART_NUM_2) // dmx uart
#define BUF_SIZE 512
uart_config_t uart_config = {
    .baud_rate = 250000,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_2,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = 0,
    .source_clk = UART_SCLK_DEFAULT,
};
static QueueHandle_t dmx_rx_queue;
static uint16_t _StartDMXAddr; // First adress liestend
static uint16_t _NbChannels; // Number of channels listened from the start address
static uint8_t * dmx_data; // stores the validated dmx data
static uint8_t * tmp_dmx_data; // stores the received dmx data
 
void createBuffer(bool TmpBufferCreate) {
    // Manage deletion of the Tmp Buffeur if not any more needed
    if((!TmpBufferCreate) && (tmp_dmx_data != NULL)) {
        free(tmp_dmx_data);
        tmp_dmx_data = NULL;
    }
    // Check if buffer is already existing
    if(dmx_data != NULL) {
        dmx_data = (uint8_t *) realloc(dmx_data, sizeof(uint8_t)*(_NbChannels+1));
    } else {
         dmx_data = (uint8_t *) malloc(sizeof(uint8_t)*(_NbChannels+1));
    }
    memset(dmx_data,0,sizeof(uint8_t)*(_NbChannels+1));
    // Manage the temporrary buffer if needed
    if(TmpBufferCreate) {
        if(tmp_dmx_data != NULL) {
            tmp_dmx_data = (uint8_t *) realloc(tmp_dmx_data, sizeof(uint8_t)*(_NbChannels+1));
        } else {
            tmp_dmx_data = (uint8_t *) malloc(sizeof(uint8_t)*(_NbChannels+1));
        }
    }
}
 
void dmx_write(uint16_t channel, uint8_t value) {
    if(channel < 1 || channel > 512) return;     // restrict acces to dmx array to valid values
    dmx_data[channel] = value;
}
 
void dmx_write_all(uint8_t * data, uint16_t start, size_t size) {
    if(start < 1 || start > 512 || start + size > 513) return; // restrict acces to dmx array to valid values
    memcpy((uint8_t *)dmx_data + start, data, size);
}
 
void uart_send_task(void*pvParameters) {
    uint8_t start_code = 0x00;
    for(;;) {
        uart_wait_tx_done(DMX_UART_NUM, 1000); // wait till uart is ready
        uart_set_line_inverse(DMX_UART_NUM, UART_SIGNAL_TXD_INV); // set line to inverse, creates break signal
        esp_rom_delay_us(184); // wait break time
        uart_set_line_inverse(DMX_UART_NUM, 0); // disable break signal
        esp_rom_delay_us(24); // wait mark after break
        //ESP_LOGI(DMX_RS485_TAG, "Write data to uart");
        uart_write_bytes(DMX_UART_NUM, (const char*) &start_code, 1); // write start code
        uart_write_bytes(DMX_UART_NUM, (const char*) dmx_data+1, 512); // transmit the dmx data
    }
}
 
void set_led(const uint8_t color_cmd) {
    uint8_t start_channel = 33;
    //uint8_t temp_brightness = 30;
    //uint8_t temp_brightness_second = 30;
    uint8_t zero_array[32] = {0};
    dmx_write_all(zero_array, start_channel, 32);
    switch (color_cmd) {
        case 1: // RED RGB: 255,0,0
            for (int i = start_channel; i <= start_channel + 32; i+=4) {
                dmx_write(i, 255);
            }
            break;
        case 2: // YELLOW RGB: 255,222,0
            for (int i = start_channel; i <= start_channel + 32; i+=4) {
                dmx_write(i, 255);
                dmx_write(i + 1, 222);
            }
            break;
        case 3: // BLUE RGB: 0,0,255
            for (int i = start_channel; i <= start_channel + 32; i+=4) {
                dmx_write(i + 2, 255);
            }
            break;
        case 4: // GREEN RGB: 0,255,0
            for (int i = start_channel; i <= start_channel + 32; i+=4) {
                dmx_write(i + 1, 255);
            }
            break;
        case 5: // MIX
            // RED
            dmx_write(start_channel, 255);
            dmx_write(start_channel + 1, 0);
            dmx_write(start_channel + 2, 0);
            dmx_write(start_channel + 4, 255);
            dmx_write(start_channel + 5, 0);
            dmx_write(start_channel + 6, 0);
            // BLUE
            dmx_write(start_channel + 8, 0);
            dmx_write(start_channel + 9, 0);
            dmx_write(start_channel + 10, 255);
            dmx_write(start_channel + 12, 0);
            dmx_write(start_channel + 13, 0);
            dmx_write(start_channel + 14, 255);
            // GREEN
            dmx_write(start_channel + 16, 0);
            dmx_write(start_channel + 17, 255);
            dmx_write(start_channel + 18, 0);
            dmx_write(start_channel + 20, 0);
            dmx_write(start_channel + 21, 255);
            dmx_write(start_channel + 22, 0);
            // YELLOW
            dmx_write(start_channel + 24, 255);
            dmx_write(start_channel + 25, 222);
            dmx_write(start_channel + 26, 0);
            dmx_write(start_channel + 28, 255);
            dmx_write(start_channel + 29, 222);
            dmx_write(start_channel + 30, 0);
            break;  
        default:
            for (int i = 1; i <= 32; i++) dmx_write(i, 10);
            break;
    }
}
 
 
/* -------------------- dout -------------------- -------------------- --------------------
*/
void set_dOut(void* arg) {
    while(1) {
        if(new_DO) {
            bool d0 = dOut & 0x0001;
            bool d1 = dOut & 0x0002;
            bool d2 = dOut & 0x0004;
            bool d3 = dOut & 0x0008;
            bool d4 = dOut & 0x0010;
            bool d5 = dOut & 0x0020;
            // bool d6 = dOut & 0x0010;
            // bool d7 = dOut & 0x0020;
            bool d8 = dOut & 0x0040;
            bool d9 = dOut & 0x0080;
            gpio_set_level(DO_0, d0);
            gpio_set_level(DO_1, d1);
            gpio_set_level(DO_2, d2);
            gpio_set_level(DO_3, d3);
            gpio_set_level(DO_4, d4);
            gpio_set_level(DO_5, d5);
            // gpio_set_level(DO_6, d6);
            // gpio_set_level(DO_7, d7);
            gpio_set_level(DO_8, d8);
            gpio_set_level(DO_9, d9);
            new_DO = false;
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(NULL);
}
 
/* -------------------- TWAI -------------------- -------------------- --------------------
*/
void init_TWAI() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(GPIO_NUM_39, GPIO_NUM_40, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    //Install TWAI driver
    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        ESP_LOGI("TWAI", "Driver installed");
    } else {
        ESP_LOGE("TWAI", "Failed to install driver");
        return;
    }
 
    //Start TWAI driver
    if (twai_start() == ESP_OK) {
        ESP_LOGI("TWAI", "Driver started");
    } else {
        ESP_LOGE("TWAI", "Failed to start driver");
        return;
    }
}

twai_message_t message = {
    // Message type and format settings
    .extd = 0,              // Standard vs extended format
    .rtr = 0,               // Data vs RTR frame
    .ss = 1,                // Whether the message is single shot (i.e., does not repeat on error)
    .self = 0,              // Whether the message is a self reception request (loopback)
    .dlc_non_comp = 0,      // DLC is less than 8
    // Message ID and payload
    .identifier = 0xAAAA,
    .data_length_code = 8,
    .data = {0, 1, 2, 3},
};
 
void transmit_TWAI(void* arg) {
    while(1) {
        //Queue message for transmission
        if (twai_transmit(&message, pdMS_TO_TICKS(1000)) == ESP_OK) {
            // ESP_LOGI("TWAI", "Message queued for transmission");
        } else {
            // ESP_LOGE("TWAI", "Failed to queue message for transmission");   
        }
		vTaskDelay(pdMS_TO_TICKS(50));
    }
    vTaskDelete(NULL);
}

void receive_TWAI(void* arg) {
	twai_message_t message_rcv;
	while(1) {
		//Receive message
		if (twai_receive(&message_rcv, pdMS_TO_TICKS(1000)) == ESP_OK) {
			for(int i = 0; i < message_rcv.data_length_code; i++) {
				// ESP_LOGI("TWAI", "Message data[%d]: %d", i, message_rcv.data[i]);
				message.data[i] = message_rcv.data[i];
			}

		} 
	}
	vTaskDelete(NULL);
}
 
/* -------------------- main -------------------- -------------------- --------------------
*/
void app_main(void) {
 
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
    gpio_set_pull_mode(I2C_SDA, GPIO_PULLUP_ONLY); //I2C
    gpio_set_pull_mode(I2C_SCL, GPIO_PULLUP_ONLY); //I2C
    vTaskDelay(pdMS_TO_TICKS(50));
 
    gpio_set_level(BMS, 1);
    gpio_set_level(PASS_1, 1); // Kinco enable
    gpio_set_level(PASS_2, 1); // N.C.
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(DO_0, 1); // Lidar 24V
    gpio_set_level(DO_1, 1); // LED 24V
    gpio_set_level(DO_2, 0); // Relay K2 to close Auto Charging's 48V loop
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(DO_3, 1); // Relay K3 for Kinco Power
    vTaskDelay(pdMS_TO_TICKS(50));
 
    init_TWAI();
    xTaskCreatePinnedToCore(transmit_TWAI, "transmit_TWAI", 16000, NULL, 5, NULL, 0);
	xTaskCreatePinnedToCore(receive_TWAI, "receive_TWAI", 16000, NULL, 5, NULL, 0);
 
    ESP_LOGI(BOOTKEY_TAG, "Create bootkey task");
    xTaskCreate(check_bootkey, "check_bootkey", 4096, NULL, 5, NULL);
 
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

    xTaskCreate(check_estop, "check_estop", 16000, NULL, 5, NULL);
 
    ESP_LOGI(DIDO_SPI_TAG, "Initialize spi bus");
    ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));
    ESP_LOGI(DIDO_SPI_TAG, "Add device to spi bus");
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &si8380s_1_cfg, &handle_1));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI3_HOST, &si8380s_2_cfg, &handle_2));
    xTaskCreate(readMap_DO_DI, "readMap_state", 16000, NULL, 5, NULL);
 
    ESP_LOGI(I2C_SLAVE_TAG, "Add i2c slave device");
    ESP_ERROR_CHECK(i2c_new_slave_device(&slv_conf, &slv_handle));
    xTaskCreate(i2c_task, "i2c_task", 16000, NULL, 5, NULL);
 
    ESP_LOGI(DMX_RS485_TAG, "Configure uart parameters");
    ESP_ERROR_CHECK(uart_param_config(DMX_UART_NUM, &uart_config));
    ESP_LOGI(DMX_RS485_TAG, "Set uart pins");
    ESP_ERROR_CHECK(uart_set_pin(DMX_UART_NUM, RS3_TX, RS3_RX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_LOGI(DMX_RS485_TAG, "Install uart driver");
    ESP_ERROR_CHECK(uart_driver_install(DMX_UART_NUM, BUF_SIZE * 2, BUF_SIZE * 2, 20, &dmx_rx_queue, 0));
    esp_rom_gpio_pad_select_gpio(RS3_DE);
    gpio_set_direction(RS3_DE, GPIO_MODE_OUTPUT);
    gpio_set_level(RS3_DE, 1);
    _StartDMXAddr = 1;
    _NbChannels = 512;
    createBuffer(false);
    xTaskCreate(uart_send_task, "uart_send_task", 1024, NULL, 1, NULL);
    set_led(5); //mix color
        
    xTaskCreatePinnedToCore(set_dOut, "set_dOut", 16000, NULL, 5, NULL, 0);
 
    // esp_intr_dump(NULL);
 
//  while(1) {
        // set_led(1); //red
        // vTaskDelay(pdMS_TO_TICKS(3000));
        // set_led(2); //yellow
        // vTaskDelay(pdMS_TO_TICKS(3000));
        // set_led(3); //blue
        // vTaskDelay(pdMS_TO_TICKS(3000));
        // set_led(4); //green
        // vTaskDelay(pdMS_TO_TICKS(3000));
//  }
}