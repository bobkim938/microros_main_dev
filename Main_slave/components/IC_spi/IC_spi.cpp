#include "IC_spi.h"

IC_SPI::IC_SPI(IC_SPI_Config *spi_config) {
    busESP.mosi_io_num = spi_config->mosi;
    busESP.miso_io_num = spi_config->miso;
    busESP.sclk_io_num = spi_config->sclk;
    busESP.quadwp_io_num = -1;
    busESP.quadhd_io_num = -1; 

    IC_dev.command_bits = 4; 
    IC_dev.address_bits = 4; 
    IC_dev.dummy_bits = 0;
    IC_dev.clock_speed_hz = 1000000; // 1 MHz
    IC_dev.duty_cycle_pos = 128;
    IC_dev.mode = 0;
    IC_dev.spics_io_num = spi_config->cs; 
    IC_dev.cs_ena_posttrans = 0; 
    IC_dev.queue_size = 3;
}

esp_err_t IC_SPI::begin() {
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &busESP, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI3_HOST, &IC_dev, &handle);
    if(ret != ESP_OK) return ret;
    return ret;
}

esp_err_t IC_SPI::test() {
    esp_err_t ret = read_spi(AM_IP_4kreg::CFG1_A, RD0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(0, RD1);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(AM_IP_4kreg::CFG1_B, RD0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(0, RD1);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(AM_IP_4kreg::CFG1_C, RD0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(0, RD1);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(AM_IP_4kreg::CFG1_D, RD0);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(0, RD1);
    printf("Received data: 0x%02X\n", recvbuf[0]);
    ret = read_spi(0, NOP);
    return ret;
}

esp_err_t IC_SPI::read_spi(uint8_t reg, uint8_t op_code) {
    t.length = 16;
    t.cmd = op_code;
    t.addr = HWA;
    if(reg == 0) {sendbuf[0] = 0;}
    else {sendbuf[0] = reg;}
    t.tx_buffer = sendbuf;
    t.rx_buffer = recvbuf;
    esp_err_t ret = spi_device_transmit(handle, &t);
    return ret;
}



