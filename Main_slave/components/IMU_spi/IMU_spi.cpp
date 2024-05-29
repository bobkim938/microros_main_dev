#include "IMU_SPI.h"

DK42688_SPI::DK42688_SPI(DK42688_SPI_Config *spi_config) {
    buscfg.mosi_io_num = spi_config->mosi;
    buscfg.miso_io_num = spi_config->miso;
    buscfg.sclk_io_num = spi_config->sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    devcfg.command_bits = 8;
    devcfg.address_bits = 0;
    devcfg.dummy_bits = 0;
    devcfg.clock_speed_hz = 1000000; // 1 MHz
    devcfg.duty_cycle_pos = 128;
    devcfg.mode = 3;
    devcfg.spics_io_num = spi_config->cs;
    devcfg.cs_ena_posttrans = 0; 
    devcfg.cs_ena_pretrans = 0;
    devcfg.queue_size = 5;
}   

esp_err_t DK42688_SPI::read_spi(uint8_t reg) {
    t.length = 8;
    t.cmd = reg | 0x80;
    t.tx_buffer = 0;
    t.rx_buffer = recvbuf;
    ret = spi_device_transmit(handle, &t);
    return ret;
}

esp_err_t DK42688_SPI::write_spi(uint8_t reg, uint8_t data, uint8_t len) {
    t.length = len * 8;
    t.cmd = reg;
    sendbuf[0] = data;
    t.tx_buffer = sendbuf;
    t.rx_buffer = 0;
    ret = spi_device_transmit(handle, &t);
    return ret;
}

esp_err_t DK42688_SPI::begin() {
    ret = spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI3_HOST, &devcfg, &handle);
    if(ret != ESP_OK) return ret;
    ret = who_am_i();
    if(ret != ESP_OK) return ret;
    ret = write_spi(ICM42688reg::PWR_MGMT0, 0x0f, 2); // turn on gyro and accel sensors in LN mode
    if(ret != ESP_OK) return ret;
    vTaskDelay(5);
    ret = read_spi(ICM42688reg::PWR_MGMT0);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    set_accODR(ODR::odr1k);
    vTaskDelay(1);
    set_gyro_fsr(GyroFSR::dps2000);
    vTaskDelay(1);
    set_gyroODR(ODR::odr1k);
    vTaskDelay(1);
    for(int i = 0; i < 500; i++) {
        gyro_bias[0] += get_gyro_x(1);
        gyro_bias[1] += get_gyro_y(1);
        gyro_bias[2] += get_gyro_z(1);
        acc_bias[0] += get_accel_x(1);
        acc_bias[1] += get_accel_y(1);
        acc_bias[2] += get_accel_z(1);
    }
    gyro_bias[0] /= 500.0;
    gyro_bias[1] /= 500.0;
    gyro_bias[2] /= 500.0;
    acc_bias[0] /= 500.0;
    acc_bias[1] /= 500.0;
    acc_bias[2] /= 500.0;
    acc_bias[2] -= 9.81;
    return ret;
}

esp_err_t DK42688_SPI::reset() {
    ret = write_spi(ICM42688reg::DEVICE_CONFIG, 0x01, 2);
    return ret;
}
  
esp_err_t DK42688_SPI::who_am_i() {
    ret = read_spi(ICM42688reg::WHO_AM_I);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    assert(recvbuf[0] == 0x47);
    return ret;
}

esp_err_t DK42688_SPI::set_gyro_fsr(GyroFSR fsr) {
    read_spi(ICM42688reg::GYRO_CONFIG0);
    uint8_t reg = (fsr << 5) | (recvbuf[0] & 0x1f);
    ret = write_spi(ICM42688reg::GYRO_CONFIG0, reg, 2);
    switch(fsr) {
        case GyroFSR::dps2000:
            gyro_fsr = 2000.0;
            break;
        case GyroFSR::dps1000:
            gyro_fsr = 1000.0;
            break;
        case GyroFSR::dps500:
            gyro_fsr = 500.0;
            break;
        case GyroFSR::dps250:
            gyro_fsr = 250.0;
            break;
        case GyroFSR::dps125:
            gyro_fsr = 125.0;
            break;
        case GyroFSR::dps62_5:
            gyro_fsr = 62.5;
            break;
        case GyroFSR::dps31_25:
            gyro_fsr = 31.25;
            break;
        case GyroFSR::dps15_625:
            gyro_fsr = 15.625;
            break;
    }
    read_spi(ICM42688reg::GYRO_CONFIG0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t DK42688_SPI::set_accel_fsr(AccelFSR fsr) {
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    uint8_t reg = (fsr << 5) | (recvbuf[0] & 0x1f);
    ret = write_spi(ICM42688reg::ACCEL_CONFIG0, reg, 2);
    switch(fsr) {
        case AccelFSR::g16:
            accel_fsr = 16.0;
            break;
        case AccelFSR::g8:
            accel_fsr = 8.0;
            break;
        case AccelFSR::g4:
            accel_fsr = 4.0;
            break;
        case AccelFSR::g2:
            accel_fsr = 2.0;
            break;
    }
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t DK42688_SPI::set_accODR(ODR odr) {
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    uint8_t reg = (recvbuf[0] & 0xF0) | odr;
    ret = write_spi(ICM42688reg::ACCEL_CONFIG0, reg, 2);
    read_spi(ICM42688reg::ACCEL_CONFIG0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t DK42688_SPI::set_gyroODR(ODR odr) {
    read_spi(ICM42688reg::GYRO_CONFIG0);
    uint8_t reg = (recvbuf[0] & 0xF0) | odr;
    ret = write_spi(ICM42688reg::GYRO_CONFIG0, reg, 2);
    read_spi(ICM42688reg::GYRO_CONFIG0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    return ret;
}

double DK42688_SPI::get_accel_x(uint8_t ac_flg) {
    read_spi(ICM42688reg::ACCEL_DATA_X0);
    int16_t accel_data_x0 = recvbuf[0];
    read_spi(ICM42688reg::ACCEL_DATA_X1);
    int16_t accel_data_x1 = recvbuf[0];
    int16_t accel_x_raw = (accel_data_x1 << 8) | accel_data_x0;
    double acc_x = (accel_x_raw * accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_x -= acc_bias[0];
    }
    return acc_x;
}

double DK42688_SPI::get_accel_y(uint8_t ac_flg) {
    read_spi(ICM42688reg::ACCEL_DATA_Y0);
    int16_t accel_data_y0 = recvbuf[0];
    read_spi(ICM42688reg::ACCEL_DATA_Y1);
    int16_t accel_data_y1 = recvbuf[0];
    int16_t accel_y_raw = (accel_data_y1 << 8) | accel_data_y0;
    double acc_y = (accel_y_raw * accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_y -= acc_bias[1];
    }
    return acc_y;
}

double DK42688_SPI::get_accel_z(uint8_t ac_flg) {
    read_spi(ICM42688reg::ACCEL_DATA_Z0);
    int16_t accel_data_z0 = recvbuf[0];
    read_spi(ICM42688reg::ACCEL_DATA_Z1);
    int16_t accel_data_z1 = recvbuf[0];
    int16_t accel_z_raw = (accel_data_z1 << 8) | accel_data_z0;
    double acc_z = (accel_z_raw * accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_z -= acc_bias[2];
    }
    return acc_z;
}

double DK42688_SPI::get_gyro_x(uint8_t gb_flg) {
    read_spi(ICM42688reg::GYRO_DATA_X0);
    int16_t gyro_data_x0 = recvbuf[0];  
    read_spi(ICM42688reg::GYRO_DATA_X1);
    int16_t gyro_data_x1 = recvbuf[0];  
    int16_t gyro_x_raw = (gyro_data_x1 << 8) | gyro_data_x0;  
    double gyro_x = gyro_x_raw * (gyro_fsr / 32768.0);  
    if(gb_flg == 0) {
        gyro_x -= gyro_bias[0];
    }
    return (gyro_x * 3.14159/180.0);
}

double DK42688_SPI::get_gyro_y(uint8_t gb_flg) {
    read_spi(ICM42688reg::GYRO_DATA_Y0);
    int16_t gyro_data_y0 = recvbuf[0];  
    read_spi(ICM42688reg::GYRO_DATA_Y1);
    int16_t gyro_data_y1 = recvbuf[0];  
    int16_t gyro_y_raw = (gyro_data_y1 << 8) | gyro_data_y0;  
    double gyro_y = gyro_y_raw * (gyro_fsr / 32768.0);  
    if(gb_flg == 0) {
        gyro_y -= gyro_bias[1];
    }
    return (gyro_y * 3.14159/180.0);
}

double DK42688_SPI::get_gyro_z(uint8_t gb_flg) {
    read_spi(ICM42688reg::GYRO_DATA_Z0);
    int16_t gyro_data_z0 = recvbuf[0]; 
    read_spi(ICM42688reg::GYRO_DATA_Z1);
    int16_t gyro_data_z1 = recvbuf[0]; 
    int16_t gyro_z_raw = (gyro_data_z1 << 8) | gyro_data_z0;  
    double gyro_z = gyro_z_raw * (gyro_fsr / 32768.0);  
    if(gb_flg == 0) {
        gyro_z -= gyro_bias[2];
    }
    return (gyro_z * 3.14159/180.0);
}

int16_t DK42688_SPI::get_ax0() {
    read_spi(ICM42688reg::ACCEL_DATA_X0);
    int16_t accel_data_x0 = recvbuf[0];
    return accel_data_x0;
}

int16_t DK42688_SPI::get_ax1() {
    read_spi(ICM42688reg::ACCEL_DATA_X1);
    int16_t accel_data_x1 = recvbuf[0];
    return accel_data_x1;
}

int16_t DK42688_SPI::get_ay0() {
    read_spi(ICM42688reg::ACCEL_DATA_Y0);
    int16_t accel_data_y0 = recvbuf[0];
    return accel_data_y0;
}

int16_t DK42688_SPI::get_ay1() {
    read_spi(ICM42688reg::ACCEL_DATA_Y1);
    int16_t accel_data_y1 = recvbuf[0];
    return accel_data_y1;
}

int16_t DK42688_SPI::get_az0() {
    read_spi(ICM42688reg::ACCEL_DATA_Z0);
    int16_t accel_data_z0 = recvbuf[0];
    return accel_data_z0;
}

int16_t DK42688_SPI::get_az1() {
    read_spi(ICM42688reg::ACCEL_DATA_Z1);
    int16_t accel_data_z1 = recvbuf[0];
    return accel_data_z1;
}

int16_t DK42688_SPI::get_gx0() {
    read_spi(ICM42688reg::GYRO_DATA_X0);
    int16_t gyro_data_x0 = recvbuf[0];
    return gyro_data_x0;
}

int16_t DK42688_SPI::get_gx1() {
    read_spi(ICM42688reg::GYRO_DATA_X1);
    int16_t gyro_data_x1 = recvbuf[0];
    return gyro_data_x1;
}

int16_t DK42688_SPI::get_gy0() {
    read_spi(ICM42688reg::GYRO_DATA_Y0);
    int16_t gyro_data_y0 = recvbuf[0];
    return gyro_data_y0;
}

int16_t DK42688_SPI::get_gy1() {
    read_spi(ICM42688reg::GYRO_DATA_Y1);
    int16_t gyro_data_y1 = recvbuf[0];
    return gyro_data_y1;
}

int16_t DK42688_SPI::get_gz0() {
    read_spi(ICM42688reg::GYRO_DATA_Z0);
    int16_t gyro_data_z0 = recvbuf[0];
    return gyro_data_z0;
}

int16_t DK42688_SPI::get_gz1() {
    read_spi(ICM42688reg::GYRO_DATA_Z1);
    int16_t gyro_data_z1 = recvbuf[0];
    return gyro_data_z1;
}





