/*
    Copyright (c) 2021 Alethea Katherine Flowers.
    Published under the standard MIT License.
    Full text available at: https://opensource.org/licenses/MIT
*/

#pragma once

/* SERCOM SPI driver for SAMD51 */

#include "sam.h"
#include "sercom.h"
#include <stddef.h>

typedef struct {
	SercomSpi* sercom;
	uint8_t dopo;
	uint8_t dipo;
	uint8_t phase;
	uint8_t polarity;

	uint8_t mosi;
	uint8_t mosi_alt;
	
	uint8_t miso;
	uint8_t miso_alt;

	uint8_t sck;
	uint8_t sck_alt;
	
	uint8_t cs;
} SPIConfig;

extern const SPIConfig SPI_CONF_TENSION_A;
extern const SPIConfig SPI_CONF_TENSION_B;
extern const SPIConfig SPI_CONF_BLDC_A;
extern const SPIConfig SPI_CONF_BLDC_B;
extern const SPIConfig SPI_CONF_BLDC_C;

void spi_init(const SPIConfig* inst);
uint16_t spi_write_read16(const SPIConfig* inst, uint16_t data);
void spi_change_mode(const SPIConfig* inst);

