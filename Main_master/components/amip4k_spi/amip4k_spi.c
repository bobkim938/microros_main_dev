#include "amip4k_spi.h"

esp_err_t amip4k_spi_begin(sboalbot_amip4k* obj) {
    obj->HWA = 0b0000;
    obj->sendbuf[0] = 0;
    obj->recvbuf[0] = 0;
    obj->MVAL = 0;
    obj->CNT = 0;
    obj->POSIT = 0;
    obj->CFG1[0] = 0x17;
    obj->CFG1[1] = 0x09;
    obj->CFG1[2] = 0xFF;
    obj->CFG1[3] = 0x00;
    obj->CFG2[0] = 0x66;
    obj->CFG2[1] = 0x02;
    obj->CFG2[2] = 0x80;
    obj->CFG2[3] = 0xC5;
    obj->CFG3[0] = 0x00;
    obj->CFG3[1] = 0x00;
    obj->CFG3[2] = 0x00;
    obj->CFG3[3] = 0x00;
    obj->busESP.miso_io_num = SPI_MISO;
    obj->busESP.mosi_io_num = SPI_MOSI;
    obj->busESP.sclk_io_num = SPI_SCLK;
    obj->busESP.quadwp_io_num = -1;
    obj->busESP.quadhd_io_num = -1;

    obj->IC_R.command_bits = 0;
    obj->IC_R.address_bits = 0;
    obj->IC_R.dummy_bits = 0;
    obj->IC_R.clock_speed_hz = 1000000; // 1MHz
    obj->IC_R.duty_cycle_pos = 128;
    obj->IC_R.mode = 0;
    obj->IC_R.spics_io_num = SPI_CS_R;
    obj->IC_R.cs_ena_pretrans = 0.5;
    obj->IC_R.cs_ena_posttrans = 1;
    obj->IC_R.queue_size = 5;

    obj->IC_L.command_bits = 0;
    obj->IC_L.address_bits = 0;
    obj->IC_L.dummy_bits = 0;
    obj->IC_L.clock_speed_hz = 1000000; // 1MHz
    obj->IC_L.duty_cycle_pos = 128;
    obj->IC_L.mode = 0;
    obj->IC_L.spics_io_num = SPI_CS_L;
    obj->IC_L.cs_ena_pretrans = 0.5;
    obj->IC_L.cs_ena_posttrans = 1;
    obj->IC_L.queue_size = 5;


    esp_err_t ret;
    ret = spi_bus_add_device(SPI2_HOST, &obj->IC_R, &obj->handle_IC_R);
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI2_HOST, &obj->IC_L, &obj->handle_IC_L);
    if(ret != ESP_OK) return ret;
    // CHECK STAT ID
    ret = amip4k_spi_read_spi(obj, AMIP4K_STAT_A, RD0, obj->handle_IC_R);
    ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    assert(obj->recvbuf[0] == 0x4300); // can change to WHILE LOOP

    ret = amip4k_spi_read_spi(obj, AMIP4K_STAT_A, RD0, obj->handle_IC_L);
    ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    assert(obj->recvbuf[0] == 0x4300); // can change to WHILE LOOP

    ret = amip4k_spi_rate_conf(obj, Interpolation_rate_4);
    if(ret != ESP_OK) return ret;
    ret = amip4k_spi_write_CFG3(obj, 'L', 0);
    if(ret != ESP_OK) return ret;
    ret = amip4k_spi_write_CFG3(obj, 'R', 0);
    return ret;
}


esp_err_t amip4k_spi_read_spi(sboalbot_amip4k* obj, uint8_t reg, uint8_t op_code, spi_device_handle_t handle) {
    // EACH TIME THE CLOCK CYCLE SHOULD BE ONLY 16 CYCLES
    obj->t_IC.length = 16;
    obj->sendbuf[0] = (op_code << 12) | (obj->HWA << 8) | reg;
    obj->sendbuf[0] = SPI_SWAP_DATA_TX(obj->sendbuf[0], 16);
    obj->t_IC.tx_buffer = obj->sendbuf;
    obj->t_IC.rx_buffer = obj->recvbuf;
    esp_err_t ret = spi_device_transmit(handle, &obj->t_IC);
    return ret;
}

esp_err_t amip4k_spi_write_spi(sboalbot_amip4k* obj, uint8_t reg, uint8_t op_code, spi_device_handle_t handle) {
    obj->t_IC.length = 16;
    obj->sendbuf[0] = (op_code << 12) | (obj->HWA << 8) | reg;
    obj->sendbuf[0] = SPI_SWAP_DATA_TX(obj->sendbuf[0], 16);
    obj->t_IC.tx_buffer = obj->sendbuf;
    obj->t_IC.rx_buffer = 0;
    esp_err_t ret = spi_device_transmit(handle, &obj->t_IC);
    return ret;
}

esp_err_t amip4k_spi_write_CFG1(sboalbot_amip4k* obj, char IC) {
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_A, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[0], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_B, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[1], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_C, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[2], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_D, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[3], WRD, obj->handle_IC_L); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG1\n");

        ret = amip4k_spi_read_spi(obj, AMIP4K_CFG1_A, RD0, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG1[0] = obj->recvbuf[0] & 0x00FF;
        obj->CFG1[1] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG1[2] = obj->recvbuf[0] & 0x00FF;
        obj->CFG1[3] = (obj->recvbuf[0] & 0xFF00) >> 8;
    }
    else if(IC == 'R') {
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_A, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[0], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_B, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[1], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_C, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[2], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG1_D, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG1[3], WRD, obj->handle_IC_R); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG1\n");

        ret = amip4k_spi_read_spi(obj, AMIP4K_CFG1_A, RD0, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG1[0] = obj->recvbuf[0] & 0x00FF;
        obj->CFG1[1] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG1[2] = obj->recvbuf[0] & 0x00FF;
        obj->CFG1[3] = (obj->recvbuf[0] & 0xFF00) >> 8;
    }
    return ret;
}

esp_err_t amip4k_spi_write_CFG2(sboalbot_amip4k* obj, char IC) {
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_A, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[0], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_B, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[1], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_C, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[2], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_D, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[3], WRD, obj->handle_IC_L); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG2\n");

        ret = amip4k_spi_read_spi(obj, AMIP4K_CFG2_A, RD0, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG2[0] = obj->recvbuf[0] & 0x00FF;
        obj->CFG2[1] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG2[2] = obj->recvbuf[0] & 0x00FF;
        obj->CFG2[3] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    else if(IC == 'R') {
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_A, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[0], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_B, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[1], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_C, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[2], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG2_D, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG2[3], WRD, obj->handle_IC_R); 
        if(ret != ESP_OK) return ret;

        // printf("Writing to CFG2\n");

        ret = amip4k_spi_read_spi(obj, AMIP4K_CFG2_A, RD0, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG2[0] = obj->recvbuf[0] & 0x00FF;
        obj->CFG2[1] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG2[2] = obj->recvbuf[0] & 0x00FF;
        obj->CFG2[3] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    return ret;
}

esp_err_t amip4k_spi_write_CFG3(sboalbot_amip4k* obj, char IC, bool abSwitch) {
    if(abSwitch) {
        obj->CFG3[1] &= (0b1 <<5);
    }
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_A, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[0], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_B, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[1], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_C, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[2], WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_D, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[3], WRD, obj->handle_IC_L); 
        if(ret != ESP_OK) return ret;
        
        // printf("Writing to CFG3\n");

        ret = amip4k_spi_read_spi(obj, AMIP4K_CFG3_A, RD0, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG3[0] = obj->recvbuf[0] & 0x00FF;
        obj->CFG3[1] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG3[2] = obj->recvbuf[0] & 0x00FF;
        obj->CFG3[3] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    else if(IC == 'R') {
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_A, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[0], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_B, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[1], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_C, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[2], WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, AMIP4K_CFG3_D, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, obj->CFG3[3], WRD, obj->handle_IC_R); 
        if(ret != ESP_OK) return ret;
        
        // printf("Writing to CFG3\n");

        ret = amip4k_spi_read_spi(obj, AMIP4K_CFG3_A, RD0, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG3[0] = obj->recvbuf[0] & 0x00FF;
        obj->CFG3[1] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
        ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CFG3[2] = obj->recvbuf[0] & 0x00FF;
        obj->CFG3[3] = (obj->recvbuf[0] & 0xFF00) >> 8;
        // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    }
    return ret;
}

esp_err_t amip4k_spi_rate_conf(sboalbot_amip4k* obj, uint8_t rate) {
    esp_err_t ret;
    if(rate == Interpolation_rate_16 || rate == Interpolation_rate_8 || rate == Interpolation_rate_4) {
        obj->CFG1[0] = (obj->CFG1[0] & 0b11100000) | Interpolation_rate_32;
        ret = amip4k_spi_write_CFG1(obj, 'L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG1(obj, 'R');
        if(ret != ESP_OK) return ret;
        obj->CFG2[0] = (obj->CFG2[0] & 0b00011111) | (rate << 5);
        ret = amip4k_spi_write_CFG2(obj, 'L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG2(obj, 'R');
        if(ret != ESP_OK) return ret;
    }
    else {
        obj->CFG1[0] = (obj->CFG1[0] & 0b11100000) | rate;
        ret = amip4k_spi_write_CFG1(obj, 'L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG1(obj, 'R');
        if(ret != ESP_OK) return ret;
        obj->CFG2[0] = (obj->CFG2[0] & 0b00011111);
        ret = amip4k_spi_write_CFG2(obj, 'L');
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_CFG2(obj, 'R');
        if(ret != ESP_OK) return ret;
    }
    return ret;
}

esp_err_t amip4k_spi_readSTAT(sboalbot_amip4k* obj) { 
    esp_err_t ret;
    ret = amip4k_spi_read_spi(obj, AMIP4K_STAT_A, RD0, obj->handle_IC_L);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", obj->recvbuf[0]);
    ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", obj->recvbuf[0]);

    ret = amip4k_spi_read_spi(obj, AMIP4K_STAT_A, RD0, obj->handle_IC_R);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", obj->recvbuf[0]);
    ret = amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
    obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", obj->recvbuf[0]);
    return ret;
}

int32_t amip4k_spi_readMVAL(sboalbot_amip4k* obj, char IC) { 
    if(IC == 'L') {
        amip4k_spi_read_spi(obj, AMIP4K_MVAL_A, RD0, obj->handle_IC_L);
        amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->MVAL = (obj->MVAL & 0) | obj->recvbuf[0];
        amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->MVAL |= (obj->recvbuf[0] << 16);
        obj->MVAL = obj->MVAL >> 2;
    }
    else if(IC == 'R') {
        amip4k_spi_read_spi(obj, AMIP4K_MVAL_A, RD0, obj->handle_IC_R);
        amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->MVAL = (obj->MVAL & 0) | obj->recvbuf[0];
        amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->MVAL |= (obj->recvbuf[0] << 16);
        obj->MVAL = obj->MVAL >> 2;
    }
    return obj->MVAL;
}

int32_t amip4k_spi_readCNT(sboalbot_amip4k* obj, char IC) {
    if(IC == 'L') {
        amip4k_spi_read_spi(obj, AMIP4K_CNT_A, RD0, obj->handle_IC_L);
        amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CNT = (obj->CNT & 0) | obj->recvbuf[0];
        amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CNT |= (obj->recvbuf[0] << 16);
        obj->CNT = obj->CNT >> 2;
    }
    else if(IC == 'R') {
        amip4k_spi_read_spi(obj, AMIP4K_CNT_A, RD0, obj->handle_IC_R);
        amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CNT = (obj->CNT & 0) | obj->recvbuf[0];
        amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->CNT |= (obj->recvbuf[0] << 16);
        obj->CNT = obj->CNT >> 2;
    }
    return obj->CNT;
}

int32_t amip4k_spi_readPOSIT(sboalbot_amip4k* obj, char IC) {
    if(IC == 'L') {
        amip4k_spi_read_spi(obj, AMIP4K_POSIT_A, RD0, obj->handle_IC_L);
        amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->POSIT = (obj->POSIT & 0) | obj->recvbuf[0];
        amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_L);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->POSIT |= (obj->recvbuf[0] << 16);
        obj->POSIT = obj->POSIT >> 2;
    }
    else if(IC == 'R') {
        amip4k_spi_read_spi(obj, AMIP4K_POSIT_A, RD0, obj->handle_IC_R);
        amip4k_spi_read_spi(obj, 0x00, RD1, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->POSIT = (obj->POSIT & 0) | obj->recvbuf[0];
        amip4k_spi_read_spi(obj, 0x00, NOP, obj->handle_IC_R);
        obj->recvbuf[0] = SPI_SWAP_DATA_RX(obj->recvbuf[0], 16);
        obj->POSIT |= (obj->recvbuf[0] << 16);
        obj->POSIT = obj->POSIT >> 2;
    }
    return obj->POSIT;
}

esp_err_t amip4k_spi_reset_cnt(sboalbot_amip4k* obj, char IC) {
    esp_err_t ret = ESP_OK;
    if(IC == 'L') {
        esp_err_t ret = amip4k_spi_write_spi(obj, AMIP4K_CMD_A, WRA, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, 0x01, WRD, obj->handle_IC_L);
        if(ret != ESP_OK) return ret;
    }
    else if(IC == 'R') {
        esp_err_t ret = amip4k_spi_write_spi(obj, AMIP4K_CMD_A, WRA, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
        ret = amip4k_spi_write_spi(obj, 0x01, WRD, obj->handle_IC_R);
        if(ret != ESP_OK) return ret;
    }
    return ret;
}

