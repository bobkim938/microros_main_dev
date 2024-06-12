#include <stdio.h>
#include "DI_DO_SPI.h"

DI_DO_SPI::DI_DO_SPI(DI_DO_SPI_config* spi_config) {
    buscfg.mosi_io_num = spi_config->mosi;
    buscfg.miso_io_num = spi_config->miso;
    buscfg.sclk_io_num = spi_config->sclk;
    buscfg.quadwp_io_num = -1;
    buscfg.quadhd_io_num = -1;

    devcfg.command_bits = 0;
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




