#ifndef ICM42688_SPI_H
#define ICM42688_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define SPI_CS_IMU GPIO_NUM_10

#include "register.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include "esp_system.h"
#include <math.h>

esp_err_t icm42688_spi_read_spi(uint8_t reg);
esp_err_t icm42688_spi_write_spi(uint8_t reg, uint8_t data, uint8_t len);
esp_err_t icm42688_spi_reset();
esp_err_t i2c42688_spi_who_am_i();
esp_err_t icm42688_spi_begin();
esp_err_t icm42688_spi_set_gyro_fsr(uint8_t fsr);
esp_err_t icm42688_spi_set_accel_fsr(uint8_t fsr);
esp_err_t icm42688_spi_set_accODR(uint8_t odr);
esp_err_t icm42688_spi_set_gyroODR(uint8_t odr);
esp_err_t icm42688_spi_set_nf_aaf(bool nf_mode, bool aaf_mode);
esp_err_t icm42688_spi_set_gyroNF_freq(double freq);
esp_err_t icm42688_spi_set_gyroNF_bw(uint8_t bw);
esp_err_t icm42688_spi_set_aaf_bandwidth(uint8_t bandwidth);
esp_err_t icm42688_spi_set_ui_filter(uint8_t filter_order, uint8_t filter_index);
double icm42688_spi_get_accel_x(uint8_t ac_flg);
double icm42688_spi_get_accel_y(uint8_t ac_flg);
double icm42688_spi_get_accel_z(uint8_t ac_flg);
double icm42688_spi_get_gyro_x(uint8_t gb_flg);
double icm42688_spi_get_gyro_y(uint8_t gb_flg);
double icm42688_spi_get_gyro_z(uint8_t gb_flg);
int16_t icm42688_spi_get_ax0();
int16_t icm42688_spi_get_ax1();
int16_t icm42688_spi_get_ay0();
int16_t icm42688_spi_get_ay1();
int16_t icm42688_spi_get_az0();
int16_t icm42688_spi_get_az1();
int16_t icm42688_spi_get_gx0();
int16_t icm42688_spi_get_gx1();
int16_t icm42688_spi_get_gy0();
int16_t icm42688_spi_get_gy1();
int16_t icm42688_spi_get_gz0();
int16_t icm42688_spi_get_gz1();



#ifdef __cplusplus
}
#endif 


#endif // ICM42688_SPI_H