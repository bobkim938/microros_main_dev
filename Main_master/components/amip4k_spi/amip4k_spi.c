#include "amip4k_spi.h"

uint8_t HWA = 0b0000;
uint16_t sendbuf[1] = {};
uint16_t recvbuf[1] = {};
int32_t MVAL = 0;
int32_t CNT = 0;
int32_t POSIT = 0;
// CFG SET TO InterpolationRate 4 (DEFAULT FOR OUR CASE)
uint8_t CFG1[4] = {0x17, 0x09, 0xFF, 0x00}; // stored from LSB to MSB
uint8_t CFG2[4] = {0x66, 0x02, 0x80, 0xC5};
uint8_t CFG3[4] = {0x00, 0x00, 0x00, 0x00};
spi_device_handle_t handle_IC_R;
spi_device_handle_t handle_IC_L; 
spi_transaction_t t_IC;

spi_bus_config_t busESP = {
    .mosi_io_num = SPI_MOSI,
    .miso_io_num = SPI_MISO,
    .sclk_io_num = SPI_SCLK,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
};

spi_device_interface_config_t IC_R= {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 1000000, // 1MHz
    .duty_cycle_pos = 128,
    .mode = 0,
    .spics_io_num = SPI_CS_R,
    .cs_ena_pretrans = 0.5,
    .cs_ena_posttrans = 1,
    .queue_size = 5,
};

spi_device_interface_config_t IC_L= {
    .command_bits = 0,
    .address_bits = 0,
    .dummy_bits = 0,
    .clock_speed_hz = 1000000, // 1MHz
    .duty_cycle_pos = 128,
    .mode = 0,
    .spics_io_num = SPI_CS_L,
    .cs_ena_pretrans = 0.5,
    .cs_ena_posttrans = 1,
    .queue_size = 5,
};


esp_err_t amip4k_spi_begin() {
    esp_err_t ret;
    ret = spi_bus_add_device(SPI2_HOST, &IC_R, &handle_IC_R);
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI2_HOST, &IC_L, &handle_IC_L);
    if(ret != ESP_OK) return ret;
    // CHECK STAT ID
    ret = amip4k_spi_read_spi(AMIP4K_STAT_A, RD0, handle_IC_R);
    ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    assert(recvbuf[0] == 0x4300); // can change to WHILE LOOP

    ret = amip4k_spi_read_spi(AMIP4K_STAT_A, RD0, handle_IC_L);
    ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    assert(recvbuf[0] == 0x4300); // can change to WHILE LOOP

    ret = amip4k_spi_rate_conf(Interpolation_rate_4);
    if(ret != ESP_OK) return ret;
    ret = amip4k_spi_write_CFG3('L', 0);
    if(ret != ESP_OK) return ret;
    ret = amip4k_spi_write_CFG3('R', 0);
    return ret;
}


esp_err_t amip4k_spi_read_spi(uint8_t reg, uint8_t op_code, spi_device_handle_t handle) {
    // EACH TIME THE CLOCK CYCLE SHOULD BE ONLY 16 CYCLES
    t_IC.length = 16;
    sendbuf[0] = (op_code << 12) | (HWA << 8) | reg;
    sendbuf[0] = SPI_SWAP_DATA_TX(sendbuf[0], 16);
    t_IC.tx_buffer = sendbuf;
    t_IC.rx_buffer = recvbuf;
    esp_err_t ret = spi_device_transmit(handle, &t_IC);
    return ret;
}

esp_err_t amip4k_spi_write_spi(uint8_t reg, uint8_t op_code, spi_device_handle_t handle) {
    t_IC.length = 16;
    sendbuf[0] = (op_code << 12) | (HWA << 8) | reg;
    sendbuf[0] = SPI_SWAP_DATA_TX(sendbuf[0], 16);
    t_IC.tx_buffer = sendbuf;
    t_IC.rx_buffer = 0;
    esp_err_t ret = spi_device_transmit(handle, &t_IC);
    return ret;
}

esp_err_t amip4k_spi_write_CFG1(char IC) {
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_A, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[0], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_B, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[1], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_C, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[2], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_D, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[3], WRD, handle_IC_L); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG1\n");

        ret = amip4k_spi_read_spi(AMIP4K_CFG1_A, RD0, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG1[0] = recvbuf[0] & 0x00FF;
        CFG1[1] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG1[2] = recvbuf[0] & 0x00FF;
        CFG1[3] = (recvbuf[0] & 0xFF00) >> 8;
    }
    else if(IC == 'R') {
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_A, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[0], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_B, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[1], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_C, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[2], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG1_D, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG1[3], WRD, handle_IC_R); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG1\n");

        ret = amip4k_spi_read_spi(AMIP4K_CFG1_A, RD0, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG1[0] = recvbuf[0] & 0x00FF;
        CFG1[1] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG1[2] = recvbuf[0] & 0x00FF;
        CFG1[3] = (recvbuf[0] & 0xFF00) >> 8;
    }
    return ret;
}

esp_err_t amip4k_spi_write_CFG2(char IC) {
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_A, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[0], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_B, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[1], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_C, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[2], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_D, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[3], WRD, handle_IC_L); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG2\n");

        ret = amip4k_spi_read_spi(AMIP4K_CFG2_A, RD0, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG2[0] = recvbuf[0] & 0x00FF;
        CFG2[1] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG2[2] = recvbuf[0] & 0x00FF;
        CFG2[3] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    else if(IC == 'R') {
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_A, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[0], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_B, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[1], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_C, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[2], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG2_D, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG2[3], WRD, handle_IC_R); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG2\n");

        ret = amip4k_spi_read_spi(AMIP4K_CFG2_A, RD0, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG2[0] = recvbuf[0] & 0x00FF;
        CFG2[1] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG2[2] = recvbuf[0] & 0x00FF;
        CFG2[3] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    return ret;
}

esp_err_t amip4k_spi_write_CFG3(char IC, bool abSwitch) {
    if(abSwitch) {
        CFG3[1] &= (0b1 <<5);
    }
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_A, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[0], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_B, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[1], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_C, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[2], WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_D, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[3], WRD, handle_IC_L); 
        if(ret != ESP_OK) return ret;
        
        // printf("Writing to CFG3\n");

        ret = amip4k_spi_read_spi(AMIP4K_CFG3_A, RD0, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG3[0] = recvbuf[0] & 0x00FF;
        CFG3[1] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG3[2] = recvbuf[0] & 0x00FF;
        CFG3[3] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    else if(IC == 'R') {
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_A, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[0], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_B, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[1], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_C, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[2], WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(AMIP4K_CFG3_D, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(CFG3[3], WRD, handle_IC_R); 
        if(ret != ESP_OK) return ret;
        
        // printf("Writing to CFG3\n");

        ret = amip4k_spi_read_spi(AMIP4K_CFG3_A, RD0, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG3[0] = recvbuf[0] & 0x00FF;
        CFG3[1] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CFG3[2] = recvbuf[0] & 0x00FF;
        CFG3[3] = (recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    return ret;
}

esp_err_t amip4k_spi_rate_conf(uint8_t rate) {
    esp_err_t ret;
    if(rate == Interpolation_rate_16 || rate == Interpolation_rate_8 || rate == Interpolation_rate_4) {
        CFG1[0] = (CFG1[0] & 0b11100000) | Interpolation_rate_32;
        ret = amip4k_spi_write_CFG1('L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG1('R');
        if(ret != ESP_OK) return ret;
        CFG2[0] = (CFG2[0] & 0b00011111) | (rate << 5);
        ret = amip4k_spi_write_CFG2('L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG2('R');
        if(ret != ESP_OK) return ret;
    }
    else {
        CFG1[0] = (CFG1[0] & 0b11100000) | rate;
        ret = amip4k_spi_write_CFG1('L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG1('R');
        if(ret != ESP_OK) return ret;
        CFG2[0] = (CFG2[0] & 0b00011111);
        ret = amip4k_spi_write_CFG2('L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG2('R');
        if(ret != ESP_OK) return ret;
    }
    return ret;
}

esp_err_t amip4k_spi_readSTAT() { 
    esp_err_t ret;
    ret = amip4k_spi_read_spi(AMIP4K_STAT_A, RD0, handle_IC_L);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", recvbuf[0]);

    ret = amip4k_spi_read_spi(AMIP4K_STAT_A, RD0, handle_IC_R);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

int32_t amip4k_spi_readMVAL(char IC) { 
    if(IC == 'L') {
        amip4k_spi_read_spi(AMIP4K_MVAL_A, RD0, handle_IC_L);
        amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        MVAL = (MVAL & 0) | recvbuf[0];
        amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        MVAL |= (recvbuf[0] << 16);
        MVAL = MVAL >> 2;
    }
    else if(IC == 'R') {
        amip4k_spi_read_spi(AMIP4K_MVAL_A, RD0, handle_IC_R);
        amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        MVAL = (MVAL & 0) | recvbuf[0];
        amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        MVAL |= (recvbuf[0] << 16);
        MVAL = MVAL >> 2;
    }
    return MVAL;
}

int32_t amip4k_spi_readCNT(char IC) {
    if(IC == 'L') {
        amip4k_spi_read_spi(AMIP4K_CNT_A, RD0, handle_IC_L);
        amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CNT = (CNT & 0) | recvbuf[0];
        amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CNT |= (recvbuf[0] << 16);
        CNT = CNT >> 2;
    }
    else if(IC == 'R') {
        amip4k_spi_read_spi(AMIP4K_CNT_A, RD0, handle_IC_R);
        amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CNT = (CNT & 0) | recvbuf[0];
        amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        CNT |= (recvbuf[0] << 16);
        CNT = CNT >> 2;
    }
    return CNT;
}

int32_t amip4k_spi_readPOSIT(char IC) {
    if(IC == 'L') {
        amip4k_spi_read_spi(AMIP4K_POSIT_A, RD0, handle_IC_L);
        amip4k_spi_read_spi(0x00, RD1, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        POSIT = (POSIT & 0) | recvbuf[0];
        amip4k_spi_read_spi(0x00, NOP, handle_IC_L);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        POSIT |= (recvbuf[0] << 16);
        POSIT = POSIT >> 2;
    }
    else if(IC == 'R') {
        amip4k_spi_read_spi(AMIP4K_POSIT_A, RD0, handle_IC_R);
        amip4k_spi_read_spi(0x00, RD1, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        POSIT = (POSIT & 0) | recvbuf[0];
        amip4k_spi_read_spi(0x00, NOP, handle_IC_R);
        recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
        POSIT |= (recvbuf[0] << 16);
        POSIT = POSIT >> 2;
    }
    return POSIT;
}

esp_err_t amip4k_spi_reset_cnt(char IC) {
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        esp_err_t ret = amip4k_spi_write_spi(AMIP4K_CMD_A, WRA, handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(0x01, WRD, handle_IC_L);
        if(ret != ESP_OK) return ret;
    }
    else if(IC == 'R') {
        esp_err_t ret = amip4k_spi_write_spi(AMIP4K_CMD_A, WRA, handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(0x01, WRD, handle_IC_R);
        if(ret != ESP_OK) return ret;
    }
    return ret;
}

