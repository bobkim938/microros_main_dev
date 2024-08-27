#ifndef AMIP4K_SPI_H
#define AMIP4K_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "register.h"
#include "esp_log.h"
// #include <inttypes.h>

#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define SPI_CS_R GPIO_NUM_35
#define SPI_CS_L GPIO_NUM_9

#define WRA 0b1000 // Write Address (0x8+address) 0b1000
#define WRD 0b1010 // Write Data (0xA+data) 0b1010
#define RD0 0b1100 // Read bytes 0 + 1 (2LSB) (0xC+address) 0b1100
#define RD1 0b1110 // Read Bytes 2 + 3 (2MSB) (0xE) 0b1110
#define NOP 0b0000 // Output read Register 
// #define HWA 0b0000

// AM-IP4k SPI WORD FORMAT = 4 bit OP-CODE + 4 bit ADDRESS + 8 bit DATA

// static const char *TAG1 = "IC_SPI";IC

/*SPI READ 32 bit Seq (OP-Code(4) + HWA(4) + DATA)
--------RD0 + HWA + addr -> RD1 +  HWA + 0x00 -> NOP + HWA + 0x00--------
*/

/*SPI Write 8 bit Seq (OP-Code(4) + HWA(4) + DATA)
--------WRA + HWA + addr ->  -> WRD + HWA + DATA--------
//////--REPEAT ABOVE SEQUENCE FOR 32 bit WRITING--//////
*/

esp_err_t amip4k_spi_begin();
esp_err_t amip4k_spi_read_spi(uint8_t reg, uint8_t op_code, spi_device_handle_t handle);
esp_err_t amip4k_spi_write_spi(uint8_t reg, uint8_t op_code, spi_device_handle_t handle);
esp_err_t amip4k_spi_write_CFG1(char IC);
esp_err_t amip4k_spi_write_CFG2(char IC);
esp_err_t amip4k_spi_write_CFG3(char IC, bool abSwitch);
esp_err_t amip4k_spi_rate_conf(uint8_t rate);
esp_err_t amip4k_spi_readSTAT();
int32_t amip4k_spi_readMVAL(char IC);
int32_t amip4k_spi_readCNT(char IC);
int32_t amip4k_spi_readPOSIT(char IC);
esp_err_t amip4k_spi_reset_cnt(char IC);

#ifdef __cplusplus
}
#endif 


#endif // AMIP4K_SPI_H