#include "IC_spi.h"

IC_SPI::IC_SPI(IC_SPI_Config *spi_config) {
    busESP.mosi_io_num = spi_config->mosi;
    busESP.miso_io_num = spi_config->miso;
    busESP.sclk_io_num = spi_config->sclk;
    busESP.quadwp_io_num = -1;
    busESP.quadhd_io_num = -1; 

    IC_dev.command_bits = 0; 
    IC_dev.address_bits = 0; 
    IC_dev.dummy_bits = 0;
    IC_dev.clock_speed_hz = 1000000; // 1MHz
    IC_dev.duty_cycle_pos = 128;
    IC_dev.mode = 0;
    IC_dev.spics_io_num = spi_config->cs; 
    IC_dev.cs_ena_pretrans = 1;
    IC_dev.cs_ena_posttrans = 1; 
    IC_dev.queue_size = 5;
}

esp_err_t IC_SPI::begin() {
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &busESP, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI3_HOST, &IC_dev, &handle);
    if(ret != ESP_OK) return ret;
    return ret;
}

esp_err_t IC_SPI::readSTAT() { 
    esp_err_t ret = read_spi(AM_IP_4kreg::STAT_A, RD0);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

esp_err_t IC_SPI::readMVAL() { 
    esp_err_t ret = read_spi(AM_IP_4kreg::MVAL_A, RD0);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    MVAL = (MVAL & 0) | recvbuf[0];
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    MVAL |= (recvbuf[0] << 16);
    printf("MVAL: 0x%08X\n", (unsigned int)MVAL);
    uint8_t triggerValue = (MVAL >> 1) & 0x01;
    bool isError = (MVAL & 0x01) != 0;
    MVAL = MVAL >> 2;
    std::cout << "Measured Value: " << MVAL << std::endl;
    std::cout << "Trigger Value: " << static_cast<int>(triggerValue) << std::endl;
    std::cout << "Measured Value Error: " << (isError ? "Yes" : "No") << std::endl;

    return ret;
}

esp_err_t IC_SPI::write_CFG1() {
    esp_err_t ret = write_spi(AM_IP_4kreg::CFG1_A, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x17, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG1_B, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x09, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG1_C, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0xFF, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG1_D, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x00, WRD); 
    if(ret != ESP_OK) return ret;

    printf("Writing to CFG1\n");

    ret = read_spi(AM_IP_4kreg::CFG1_A, RD0);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

esp_err_t IC_SPI::write_CFG2() {
    esp_err_t ret = write_spi(AM_IP_4kreg::CFG2_A, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x66, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG2_B, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x02, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG2_C, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x80, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG2_D, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0xC5, WRD); 
    if(ret != ESP_OK) return ret;

    printf("Writing to CFG2\n");

    ret = read_spi(AM_IP_4kreg::CFG2_A, RD0);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

esp_err_t IC_SPI::write_CFG3() {
    esp_err_t ret = write_spi(AM_IP_4kreg::CFG3_A, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x04, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG3_B, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x00, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG3_C, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x00, WRD);
    if(ret != ESP_OK) return ret;
    ret = write_spi(AM_IP_4kreg::CFG3_D, WRA);
    if(ret != ESP_OK) return ret;
    ret = write_spi(0x00, WRD); 
    if(ret != ESP_OK) return ret;
    
    printf("Writing to CFG3\n");

    ret = read_spi(AM_IP_4kreg::CFG3_A, RD0);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    // printf("Received data[0]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[1]: 0x%04X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    recvbuf[0] = SPI_SWAP_DATA_RX(recvbuf[0], 16);
    printf("Received data[2]: 0x%04X\n", recvbuf[0]);
    return ret;
}

esp_err_t IC_SPI::read_spi(uint8_t reg, uint8_t op_code) {
    // EACH TIME THE CLOCK CYCLE SHOULD BE ONLY 16 CYCLES
    t.length = 16;
    sendbuf[0] = (op_code << 12) | (HWA << 8) | reg;
    sendbuf[0] = SPI_SWAP_DATA_TX(sendbuf[0], 16);
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;
    esp_err_t ret = spi_device_transmit(handle, &t);
    return ret;
}

esp_err_t IC_SPI::write_spi(uint8_t reg, uint8_t op_code) {
    t.length = 16;
    sendbuf[0] = (op_code << 12) | (HWA << 8) | reg;
    sendbuf[0] = SPI_SWAP_DATA_TX(sendbuf[0], 16);
    t.tx_buffer = sendbuf;
    t.rx_buffer = 0;
    esp_err_t ret = spi_device_transmit(handle, &t);
    return ret;
}



