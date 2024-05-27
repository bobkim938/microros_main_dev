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
    IC_dev.clock_speed_hz = 1000000; // 1 MHz
    IC_dev.duty_cycle_pos = 128;
    IC_dev.mode = 0;
    IC_dev.spics_io_num = spi_config->cs; 
    IC_dev.cs_ena_pretrans = 0;
    IC_dev.cs_ena_posttrans = 0; 
    IC_dev.queue_size = 3;
}

esp_err_t IC_SPI::begin() {
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &busESP, SPI_DMA_CH_AUTO);
    if(ret != ESP_OK) return ret;
    ret = spi_bus_add_device(SPI3_HOST, &IC_dev, &handle);
    if(ret != ESP_OK) return ret;
    // write_spi(AM_IP_4kreg::CFG2_B, 0, WRA);
    // write_spi(0x80, 0, WRD);
    return ret;
}

esp_err_t IC_SPI::test() { // NEED MODIFICATION
    esp_err_t ret = read_spi(AM_IP_4kreg::STAT_A, RD0);
    printf("Received data[0]: 0x%02X\n", recvbuf[0]);
    ret = read_spi(0x00, RD1);
    printf("Received data[1]: 0x%02X\n", recvbuf[0]);
    ret = read_spi(0x00, NOP);
    printf("Received data[2]: 0x%02X\n", recvbuf[0]);
    return ret;
}

esp_err_t IC_SPI::read_spi(uint8_t reg, uint8_t op_code) {
    // EACH TIME THE CLOCK CYCLE SHOULD BE ONLY 16 CYCLES
    t.length = 16;
    rdsend[0] = (op_code << 12) | (HWA << 8) | reg;
    rdsend[0] = SPI_SWAP_DATA_TX(rdsend[0], 16);
    t.tx_buffer = rdsend;
    t.rx_buffer = recvbuf;
    esp_err_t ret = spi_device_transmit(handle, &t);
    return ret;
}

esp_err_t IC_SPI::write_spi(uint8_t reg, uint8_t length, uint8_t op_code) {
    t.length = 8 * length;
    t.cmd = (op_code << 12) | (HWA << 8) | reg;
    t.tx_buffer = NULL;
    t.rx_buffer = 0;
    esp_err_t ret = spi_device_transmit(handle, &t);
    return ret;
}



