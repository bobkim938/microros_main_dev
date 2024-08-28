#include "icm42688_spi.h"

esp_err_t icm42688_spi_read_spi(shoalbot_icm42688 *obj, uint8_t reg) {
    obj->t.length = 8;
    obj->t.cmd = reg | 0x80;
    obj->t.tx_buffer = 0;
    obj->t.rx_buffer = obj->IMU_recvbuf;
    obj->spi_ret = spi_device_transmit(obj->handle, &obj->t);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_write_spi(shoalbot_icm42688 *obj, uint8_t reg, uint8_t data, uint8_t len) {
    obj->t.length = len * 8;
    obj->t.cmd = reg;
    obj->IMU_sendbuf[0] = data;
    obj->t.tx_buffer = obj->IMU_sendbuf;
    obj->t.rx_buffer = 0;
    obj->spi_ret = spi_device_transmit(obj->handle, &obj->t);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_reset(shoalbot_icm42688 *obj) {
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_DEVICE_CONFIG, 0x01, 2);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_who_am_i(shoalbot_icm42688 *obj) {
    obj->spi_ret = icm42688_spi_read_spi(obj, IMU_WHO_AM_I);
    // printf("Received data: 0x%02X\n", recvbuf[0]);
    assert(obj->IMU_recvbuf[0] == 0x47);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_begin(shoalbot_icm42688 *obj) {
    obj->IMU_sendbuf[0] = 0;
    obj->IMU_recvbuf[0] = 0;
    obj->gyro_fsr = 2000.0;
    obj->accel_fsr = 16.0;
    obj->gyro_bias[0] = 0;
    obj->gyro_bias[1] = 0;
    obj->gyro_bias[2] = 0;
    obj->acc_bias[0] = 0;
    obj->acc_bias[1] = 0;
    obj->acc_bias[2] = 0;
    obj->AAF_bitShift = 0;
    obj->AAF_deltSqr = 0;

    obj->buscfg.mosi_io_num = SPI_MOSI;
    obj->buscfg.miso_io_num = SPI_MISO;
    obj->buscfg.sclk_io_num = SPI_SCLK;
    obj->buscfg.quadwp_io_num = -1;
    obj->buscfg.quadhd_io_num = -1;

    obj->devcfg.command_bits = 8,
    obj->devcfg.address_bits = 0,
    obj->devcfg.dummy_bits = 0,
    obj->devcfg.clock_speed_hz = 1000000, // 1 MHz
    obj->devcfg.duty_cycle_pos = 128,
    obj->devcfg.mode = 3,
    obj->devcfg.spics_io_num = SPI_CS_IMU,
    obj->devcfg.cs_ena_posttrans = 0,
    obj->devcfg.cs_ena_pretrans = 0, 
    obj->devcfg.queue_size = 5,


    obj->spi_ret = spi_bus_initialize(SPI2_HOST, &obj->buscfg, SPI_DMA_CH_AUTO);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = spi_bus_add_device(SPI2_HOST, &obj->devcfg, &obj->handle);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_who_am_i(obj);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_PWR_MGMT0, 0x0F, 2); // turn on gyro and accel sensors in LN mode
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    vTaskDelay(5);
    obj->spi_ret = icm42688_spi_read_spi(obj, IMU_PWR_MGMT0);
    //printf("Received data: 0x%02X\n", IMU_recvbuf[0]);
    icm42688_spi_set_accel_fsr(obj, ACCEL_G4); vTaskDelay(1);
    icm42688_spi_set_accODR(obj, ODR_500); vTaskDelay(1);
    icm42688_spi_set_gyro_fsr(obj, GYRO_DPS250); vTaskDelay(1); //dps500
    icm42688_spi_set_gyroODR(obj, ODR_500); vTaskDelay(1);
    for(int i = 0; i < 500; i++) {
        obj->gyro_bias[0] += icm42688_spi_get_gyro_x(obj, 1);
        obj->gyro_bias[1] += icm42688_spi_get_gyro_y(obj, 1);
        obj->gyro_bias[2] += icm42688_spi_get_gyro_z(obj, 1);
        obj->acc_bias[0] += icm42688_spi_get_accel_x(obj, 1);
        obj->acc_bias[1] += icm42688_spi_get_accel_y(obj, 1);
        obj->acc_bias[2] += icm42688_spi_get_accel_z(obj, 1);
    }
    obj->gyro_bias[0] /= 500.0;
    obj->gyro_bias[1] /= 500.0;
    obj->gyro_bias[2] /= 500.0;
    obj->acc_bias[0] /= 500.0;
    obj->acc_bias[1] /= 500.0;
    obj->acc_bias[2] /= 500.0;
    obj->acc_bias[2] -= 9.81; 

    icm42688_spi_set_nf_aaf(obj, 1, 1);
    icm42688_spi_set_gyroNF_freq(obj, 1000.0);
    icm42688_spi_set_gyroNF_bw(obj, NOTCH_BW_1449); 
    icm42688_spi_set_aaf_bandwidth(obj, 63);
    icm42688_spi_set_ui_filter(obj, UI_ORDER3, 5);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_gyro_fsr(shoalbot_icm42688 *obj, uint8_t fsr) {
    icm42688_spi_read_spi(obj, IMU_GYRO_CONFIG0);
    uint8_t reg = (fsr << 5) | (obj->IMU_recvbuf[0] & 0x1f);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG0, reg, 2);
    switch(fsr) {
        case GYRO_DPS2000:
            obj->gyro_fsr = 2000.0;
            break;
        case GYRO_DPS1000:
            obj->gyro_fsr = 1000.0;
            break;
        case GYRO_DPS500:
            obj->gyro_fsr = 500.0;
            break;
        case GYRO_DPS250:
            obj->gyro_fsr = 250.0;
            break;
        case GYRO_DPS125:
            obj->gyro_fsr = 125.0;
            break;
        case GYRO_DPS62_5:
            obj->gyro_fsr = 62.5;
            break;
        case GYRO_DPS31_25:
            obj->gyro_fsr = 31.25;
            break;
        case GYRO_DPS15_625:
            obj->gyro_fsr = 15.625;
            break;
    }
    icm42688_spi_read_spi(obj, IMU_GYRO_CONFIG0);
    // printf("Received data: 0x%02X\n", IMU_recvbuf[0]);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_accel_fsr(shoalbot_icm42688 *obj, uint8_t fsr) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_CONFIG0);
    uint8_t reg = (fsr << 5) | (obj->IMU_recvbuf[0] & 0x1f);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_ACCEL_CONFIG0, reg, 2);
    switch(fsr) {
        case ACCEL_G16:
            obj->accel_fsr = 16.0;
            break;
        case ACCEL_G8:
            obj->accel_fsr = 8.0;
            break;
        case ACCEL_G4:
            obj->accel_fsr = 4.0;
            break;
        case ACCEL_G2:
            obj->accel_fsr = 2.0;
            break;
    }
    icm42688_spi_read_spi(obj, IMU_ACCEL_CONFIG0);
    // printf("Received data: 0x%02X\n", IMU_recvbuf[0]);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_accODR(shoalbot_icm42688 *obj, uint8_t odr) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_CONFIG0);
    uint8_t reg = (obj->IMU_recvbuf[0] & 0xF0) | odr;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_ACCEL_CONFIG0, reg, 2);
    icm42688_spi_read_spi(obj, IMU_ACCEL_CONFIG0);
    // printf("Received data: 0x%02X\n", IMU_recvbuf[0]);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_gyroODR(shoalbot_icm42688 *obj, uint8_t odr) {
    icm42688_spi_read_spi(obj, IMU_GYRO_CONFIG0);
    uint8_t reg = (obj->IMU_recvbuf[0] & 0xF0) | odr;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG0, reg, 2);
    icm42688_spi_read_spi(obj, IMU_GYRO_CONFIG0);
    // printf("Received data: 0x%02X\n", IMU_recvbuf[0]);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_nf_aaf(shoalbot_icm42688 *obj, bool nf_mode, bool aaf_mode) {
    if(nf_mode && aaf_mode) {
        obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC2, 0x00, 1);
    }
    else if(nf_mode && !aaf_mode) {
        obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC2, 0x02, 1);
    }
    else if(!nf_mode && aaf_mode) {
        obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC2, 0x01, 1);
    }
    else {
        obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC2, 0x03, 1);

    }
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_gyroNF_freq(shoalbot_icm42688 *obj, double freq) {
    int16_t NF_COSWZ = 0;
    uint8_t NF_COSWZ_SEL = 0;
    double coswz = cos(2 * M_PI * freq / 32.0);
    if(abs(coswz) <= 0.875) {
        NF_COSWZ = round(coswz * 256);
        NF_COSWZ_SEL = 0;
    }
    else {
        NF_COSWZ_SEL = 1;
        if(coswz > 0.875) {
            NF_COSWZ = round(8*(1-coswz)*256);
        }
        else if(coswz < -0.875) {
            NF_COSWZ = round(-8*(1+coswz)*256);
        }
    }
    uint8_t nf_coswz = NF_COSWZ >> 8;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC6, NF_COSWZ, 1);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC7, NF_COSWZ, 1);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC8, NF_COSWZ, 1);
    uint8_t CONFIG_STATIC9 = 0x00;
    CONFIG_STATIC9 = (NF_COSWZ_SEL << 5) | (NF_COSWZ_SEL << 4) | (NF_COSWZ_SEL << 3) | (nf_coswz << 2) | (nf_coswz << 1) | nf_coswz;
    //printf("CONFIG_STATIC9: 0x%02X\n", CONFIG_STATIC9);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC9, CONFIG_STATIC9, 1);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC6, NF_COSWZ, 1);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC7, NF_COSWZ, 1);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC8, NF_COSWZ, 1);
    
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_gyroNF_bw(shoalbot_icm42688 *obj, uint8_t bw) {
    uint8_t set_bw = (bw<<4) | 0x01;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC5, set_bw, 1);
    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_aaf_bandwidth(shoalbot_icm42688 *obj, uint8_t bandwidth) {
     // AAF_deltSqr = bandwidth * bandwidth;
    obj->AAF_deltSqr = 3968;
    uint8_t accel_bw = (bandwidth << 1) | 0x00;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC3, bandwidth, 1); //set gyro bandwidth for aaf
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_ACCEL_CONFIG_STATIC2, accel_bw, 1); //set accel bandwidth for aaf
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC4, obj->AAF_deltSqr, 1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_ACCEL_CONFIG_STATIC3, obj->AAF_deltSqr, 1);
    if(obj->spi_ret != ESP_OK) return  obj->spi_ret;
    uint8_t AAF_DELTSQR =  obj->AAF_deltSqr >> 8;
    if(bandwidth == 1){
        obj->AAF_bitShift = 15;
    } else if(bandwidth == 2){
        obj->AAF_bitShift = 13;
    } else if(bandwidth == 3){
        obj->AAF_bitShift = 12;
    } else if(bandwidth == 4){
        obj->AAF_bitShift = 11;
    } else if(bandwidth == 5 || bandwidth == 6){
        obj->AAF_bitShift = 10;
    } else if(bandwidth > 6 && bandwidth < 10){
        obj->AAF_bitShift = 9;
    } else if(bandwidth > 9 && bandwidth < 14){
        obj->AAF_bitShift = 8;
    } else if(bandwidth > 13 && bandwidth < 19){
        obj->AAF_bitShift = 7;
    } else if(bandwidth > 18 && bandwidth < 27){
        obj->AAF_bitShift = 6;
    } else if(bandwidth > 26 && bandwidth < 37){
        obj->AAF_bitShift = 5;
    } else if(bandwidth > 36 && bandwidth < 53){
        obj->AAF_bitShift = 4;
    } else if(bandwidth > 53 && bandwidth <= 63){
        obj->AAF_bitShift = 3;
    }
    uint8_t AAF = 0x00;
    AAF = (obj->AAF_bitShift << 4) | AAF_DELTSQR;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG_STATIC5, AAF, 1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_ACCEL_CONFIG_STATIC4, AAF, 1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;

    return obj->spi_ret;
}

esp_err_t icm42688_spi_set_ui_filter(shoalbot_icm42688 *obj, uint8_t filter_order, uint8_t filter_index) {
    obj->spi_ret = icm42688_spi_read_spi(obj, IMU_GYRO_CONFIG1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    // printf("Received data(GYRO UI): 0x%02X\n", IMU_recvbuf[0]);
    uint8_t UI_gyroOrder = obj->IMU_recvbuf[0];
    UI_gyroOrder &= ~(0b11 << 2);
    UI_gyroOrder |= (filter_order & 0b11) << 2;
    // printf("UI_gyroOrder: 0x%02X\n", UI_gyroOrder);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_CONFIG1, UI_gyroOrder, 1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_read_spi(obj, IMU_GYRO_ACCEL_CONFIG0);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    // printf("Received data(GYRO ACCEL UI): 0x%02X\n", IMU_recvbuf[0]);
    uint8_t UI_gyroIndex = obj->IMU_recvbuf[0];
    UI_gyroIndex &= ~(0b1111);
    UI_gyroIndex |= filter_index & 0b1111;
    // printf("UI_gyroIndex: 0x%02X\n", UI_gyroIndex);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_ACCEL_CONFIG0, UI_gyroIndex, 1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;

    obj->spi_ret = icm42688_spi_read_spi(obj, IMU_ACCEL_CONFIG1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    // printf("Received data(ACCEL UI): 0x%02X\n", IMU_recvbuf[0]); 
    uint8_t UI_accelOrder = obj->IMU_recvbuf[0];
    UI_accelOrder &= ~(0b11 << 3);
    UI_accelOrder |= (filter_order & 0b11) << 3;
    // printf("UI_accelOrder: 0x%02X\n", UI_accelOrder);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_ACCEL_CONFIG1, UI_accelOrder, 1);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    obj->spi_ret = icm42688_spi_read_spi(obj, IMU_GYRO_ACCEL_CONFIG0);
    if(obj->spi_ret != ESP_OK) return obj->spi_ret;
    // printf("Received data(GYRO ACCEL UI): 0x%02X\n", IMU_recvbuf[0]);
    uint8_t UI_accelIndex = obj->IMU_recvbuf[0];
    UI_accelIndex &= ~(0b1111 << 4);
    UI_accelIndex |= (filter_index & 0b1111) << 4;
    // printf("UI_accelIndex: 0x%02X\n", UI_accelIndex);
    obj->spi_ret = icm42688_spi_write_spi(obj, IMU_GYRO_ACCEL_CONFIG0, UI_accelIndex, 1);

    return obj->spi_ret;
}

double icm42688_spi_get_accel_x(shoalbot_icm42688 *obj, uint8_t ac_flg) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_X0);
    int16_t accel_data_x0 = obj->IMU_recvbuf[0];
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_X1);
    int16_t accel_data_x1 = obj->IMU_recvbuf[0];
    int16_t accel_x_raw = (accel_data_x1 << 8) | accel_data_x0;
    double acc_x = (accel_x_raw * obj->accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_x -= obj->acc_bias[0];
    }
    // printf("Accel X: %f\n", acc_x);
    return acc_x;
}

double icm42688_spi_get_accel_y(shoalbot_icm42688 *obj, uint8_t ac_flg) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Y0);
    int16_t accel_data_y0 = obj->IMU_recvbuf[0];
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Y1);
    int16_t accel_data_y1 = obj->IMU_recvbuf[0];
    int16_t accel_y_raw = (accel_data_y1 << 8) | accel_data_y0;
    double acc_y = (accel_y_raw * obj->accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_y -= obj->acc_bias[1];
    }
    // printf("Accel Y: %f\n", acc_y);
    return acc_y;
}

double icm42688_spi_get_accel_z(shoalbot_icm42688 *obj, uint8_t ac_flg) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Z0);
    int16_t accel_data_z0 = obj->IMU_recvbuf[0];
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Z1);
    int16_t accel_data_z1 = obj->IMU_recvbuf[0];
    int16_t accel_z_raw = (accel_data_z1 << 8) | accel_data_z0;
    double acc_z = (accel_z_raw * obj->accel_fsr/32768.0) * 9.81;
    if(ac_flg == 0) {
        acc_z -= obj->acc_bias[2];
    }
    // printf("Accel Z: %f\n", acc_z);
    return acc_z;
}

double icm42688_spi_get_gyro_x(shoalbot_icm42688 *obj, uint8_t gb_flg) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_X0);
    uint8_t gyro_data_x0 = obj->IMU_recvbuf[0];  
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_X1);
    uint8_t gyro_data_x1 = obj->IMU_recvbuf[0]; 
    int16_t gyro_x_raw = (gyro_data_x1 << 8) | gyro_data_x0;  
    double gyro_x = gyro_x_raw * (obj->gyro_fsr / 32768.0) * (PI / 180); 
    if(gb_flg == 0) {
        gyro_x -= obj->gyro_bias[0];
    } 
    // printf("Gyro X: %f\n", gyro_x);
    return gyro_x;
}

double icm42688_spi_get_gyro_y(shoalbot_icm42688 *obj, uint8_t gb_flg) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Y0);
    int16_t gyro_data_y0 = obj->IMU_recvbuf[0];  
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Y1);
    int16_t gyro_data_y1 = obj->IMU_recvbuf[0];  
    int16_t gyro_y_raw = (gyro_data_y1 << 8) | gyro_data_y0;  
    double gyro_y = gyro_y_raw * (obj->gyro_fsr / 32768.0)  * (PI / 180);  
    if(gb_flg == 0) {
        gyro_y -= obj->gyro_bias[1];
    }
    // printf("Gyro Y: %f\n", gyro_y);
    return gyro_y;
}

double icm42688_spi_get_gyro_z(shoalbot_icm42688 *obj, uint8_t gb_flg) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Z0);
    int16_t gyro_data_z0 = obj->IMU_recvbuf[0]; 
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Z1);
    int16_t gyro_data_z1 = obj->IMU_recvbuf[0]; 
    int16_t gyro_z_raw = (gyro_data_z1 << 8) | gyro_data_z0;  
    double gyro_z = gyro_z_raw * (obj->gyro_fsr / 32768.0)  * (PI / 180); 
    if(gb_flg == 0) {
        gyro_z -= obj->gyro_bias[2];
    } 
    // printf("Gyro Z: %f\n", gyro_z);
    return gyro_z;
}

int16_t icm42688_spi_get_ax0(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_X0);
    int16_t accel_data_x0 = obj->IMU_recvbuf[0];
    return accel_data_x0;
}

int16_t icm42688_spi_get_ax1(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_X1);
    int16_t accel_data_x1 = obj->IMU_recvbuf[0];
    return accel_data_x1;
}

int16_t icm42688_spi_get_ay0(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Y0);
    int16_t accel_data_y0 = obj->IMU_recvbuf[0];
    return accel_data_y0;
}

int16_t icm42688_spi_get_ay1(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Y1);
    int16_t accel_data_y1 = obj->IMU_recvbuf[0];
    return accel_data_y1;
}

int16_t icm42688_spi_get_az0(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Z0);
    int16_t accel_data_z0 = obj->IMU_recvbuf[0];
    return accel_data_z0;
}

int16_t icm42688_spi_get_az1(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_ACCEL_DATA_Z1);
    int16_t accel_data_z1 = obj->IMU_recvbuf[0];
    return accel_data_z1;
}

int16_t icm42688_spi_get_gx0(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_X0);
    int16_t gyro_data_x0 = obj->IMU_recvbuf[0];
    return gyro_data_x0;
}

int16_t icm42688_spi_get_gx1(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_X1);
    int16_t gyro_data_x1 = obj->IMU_recvbuf[0];
    return gyro_data_x1;
}

int16_t icm42688_spi_get_gy0(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Y0);
    int16_t gyro_data_y0 = obj->IMU_recvbuf[0];
    return gyro_data_y0;
}

int16_t icm42688_spi_get_gy1(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Y1);
    int16_t gyro_data_y1 = obj->IMU_recvbuf[0];
    return gyro_data_y1;
}

int16_t icm42688_spi_get_gz0(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Z0);
    int16_t gyro_data_z0 = obj->IMU_recvbuf[0];
    return gyro_data_z0;
}

int16_t icm42688_spi_get_gz1(shoalbot_icm42688 *obj) {
    icm42688_spi_read_spi(obj, IMU_GYRO_DATA_Z1);
    int16_t gyro_data_z1 = obj->IMU_recvbuf[0];
    return gyro_data_z1;
}