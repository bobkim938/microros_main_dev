#ifndef ICM42688_SPI_H
#define ICM42688_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#define SPI_MOSI GPIO_NUM_11 
#define SPI_SCLK GPIO_NUM_12
#define SPI_MISO GPIO_NUM_13 
#define SPI_CS_IMU GPIO_NUM_10

#define PI 3.14159265359

#include "register.h"
#include "esp_err.h"
#include "driver/spi_master.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include "esp_system.h"
#include <math.h>

typedef struct {
    uint8_t IMU_sendbuf[1];
    uint8_t IMU_recvbuf[1];
    float gyro_fsr;
    float accel_fsr;
    double gyro_bias[3];
    double acc_bias[3];
    uint8_t AAF_bitShift;
    uint16_t AAF_deltSqr;
    spi_bus_config_t buscfg;
    spi_device_interface_config_t devcfg;
    esp_err_t spi_ret;
    spi_device_handle_t handle;
    spi_transaction_t t;
} shoalbot_icm42688;

esp_err_t icm42688_spi_read_spi(shoalbot_icm42688 *obj, uint8_t reg);
esp_err_t icm42688_spi_write_spi(shoalbot_icm42688 *obj, uint8_t reg, uint8_t data, uint8_t len);
esp_err_t icm42688_spi_reset(shoalbot_icm42688 *obj);
esp_err_t i2c42688_spi_who_am_i(shoalbot_icm42688 *obj);
esp_err_t icm42688_spi_begin(shoalbot_icm42688 *obj);
esp_err_t icm42688_spi_set_gyro_fsr(shoalbot_icm42688 *obj, uint8_t fsr);
esp_err_t icm42688_spi_set_accel_fsr(shoalbot_icm42688 *obj, uint8_t fsr);
esp_err_t icm42688_spi_set_accODR(shoalbot_icm42688 *obj, uint8_t odr);
esp_err_t icm42688_spi_set_gyroODR(shoalbot_icm42688 *obj, uint8_t odr);
esp_err_t icm42688_spi_set_nf_aaf(shoalbot_icm42688 *obj, bool nf_mode, bool aaf_mode);
esp_err_t icm42688_spi_set_gyroNF_freq(shoalbot_icm42688 *obj, double freq);
esp_err_t icm42688_spi_set_gyroNF_bw(shoalbot_icm42688 *obj, uint8_t bw);
esp_err_t icm42688_spi_set_aaf_bandwidth(shoalbot_icm42688 *obj, uint8_t bandwidth);
esp_err_t icm42688_spi_set_ui_filter(shoalbot_icm42688 *obj, uint8_t filter_order, uint8_t filter_index);
double icm42688_spi_get_accel_x(shoalbot_icm42688 *obj, uint8_t ac_flg);
double icm42688_spi_get_accel_y(shoalbot_icm42688 *obj, uint8_t ac_flg);
double icm42688_spi_get_accel_z(shoalbot_icm42688 *obj, uint8_t ac_flg);
double icm42688_spi_get_gyro_x(shoalbot_icm42688 *obj, uint8_t gb_flg);
double icm42688_spi_get_gyro_y(shoalbot_icm42688 *obj, uint8_t gb_flg);
double icm42688_spi_get_gyro_z(shoalbot_icm42688 *obj, uint8_t gb_flg);
int16_t icm42688_spi_get_ax0(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_ax1(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_ay0(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_ay1(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_az0(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_az1(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_gx0(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_gx1(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_gy0(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_gy1(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_gz0(shoalbot_icm42688 *obj);
int16_t icm42688_spi_get_gz1(shoalbot_icm42688 *obj);



#ifdef __cplusplus
}
#endif 


#endif // ICM42688_SPI_H