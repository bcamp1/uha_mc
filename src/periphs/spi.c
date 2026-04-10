/*
    Copyright (c) 2021 Alethea Katherine Flowers.
    Published under the standard MIT License.
    Full text available at: https://opensource.org/licenses/MIT
*/

#include "spi.h"
#include "gpio.h"
#include "uart.h"
#include "../drivers/delay.h"
#include "../board.h"

/* 12 MHz clock for SPI */
#define SERCOM_SPI_GCLK GCLK_PCHCTRL_GEN_GCLK1;
#define SPI_BAUD_RATE (80)
#define CS_DELAY (0x20)

const SPIConfig SPI_CONF_TENSION_A = {
	.sercom = (SercomSpi*) SERCOM3,
	.dopo = 0,
	.dipo = 2,

	.polarity = 0,
	.phase = 1,

	.mosi = PIN_MOSI,
	.mosi_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.miso = PIN_MISO,
	.miso_alt = GPIO_ALTERNATE_D_SERCOM_ALT,

	.sck = PIN_SCK,
	.sck_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.cs = PIN_TENSION_A_CS,
};

const SPIConfig SPI_CONF_TENSION_B = {
	.sercom = (SercomSpi*) SERCOM3,
	.dopo = 0,
	.dipo = 2,

	.polarity = 0,
	.phase = 1,

	.mosi = PIN_MOSI,
	.mosi_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.miso = PIN_MISO,
	.miso_alt = GPIO_ALTERNATE_D_SERCOM_ALT,

	.sck = PIN_SCK,
	.sck_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.cs = PIN_TENSION_B_CS,
};

const SPIConfig SPI_CONF_BLDC_A = {
	.sercom = (SercomSpi*) SERCOM3,
	.dopo = 0,
	.dipo = 2,

	.polarity = 1,
	.phase = 0,

	.mosi = PIN_MOSI,
	.mosi_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.miso = PIN_MISO,
	.miso_alt = GPIO_ALTERNATE_D_SERCOM_ALT,

	.sck = PIN_SCK,
	.sck_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.cs = PIN_BLDC_A_CS,
};

const SPIConfig SPI_CONF_BLDC_B = {
	.sercom = (SercomSpi*) SERCOM3,
	.dopo = 0,
	.dipo = 2,

	.polarity = 1,
	.phase = 0,

	.mosi = PIN_MOSI,
	.mosi_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.miso = PIN_MISO,
	.miso_alt = GPIO_ALTERNATE_D_SERCOM_ALT,

	.sck = PIN_SCK,
	.sck_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.cs = PIN_BLDC_B_CS,
};

const SPIConfig SPI_CONF_BLDC_C = {
	.sercom = (SercomSpi*) SERCOM3,
	.dopo = 0,
	.dipo = 2,

	.polarity = 1,
	.phase = 0,

	.mosi = PIN_MOSI,
	.mosi_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.miso = PIN_MISO,
	.miso_alt = GPIO_ALTERNATE_D_SERCOM_ALT,

	.sck = PIN_SCK,
	.sck_alt = GPIO_ALTERNATE_D_SERCOM_ALT,
	
	.cs = PIN_BLDC_C_CS,
};

void spi_init(const SPIConfig* inst) {
	/* Enable clocks */
	wntr_sercom_init_clock((Sercom*)inst->sercom, GCLK_PCHCTRL_GEN_GCLK1);

	/* Reset and configure */
	inst->sercom->CTRLA.bit.ENABLE = 0;
	while (inst->sercom->SYNCBUSY.bit.ENABLE) {};

	inst->sercom->CTRLA.bit.SWRST = 1;
	while (inst->sercom->SYNCBUSY.bit.SWRST || inst->sercom->CTRLA.bit.SWRST) {};

	/* Setup SPI controller and mode (0x3 = controller) */
	inst->sercom->CTRLA.reg = (SERCOM_SPI_CTRLA_MODE(0x3) | SERCOM_SPI_CTRLA_DOPO(inst->dopo) | SERCOM_SPI_CTRLA_DIPO(inst->dipo));

	if (inst->phase) {
		inst->sercom->CTRLA.bit.CPHA = 1;
	}
	if (inst->polarity) {
		inst->sercom->CTRLA.bit.CPOL = 1;
	}

	/* Set baud to max (GCLK / 2) 6 MHz */
	inst->sercom->BAUD.reg = SERCOM_SPI_BAUD_BAUD(80);
    // 30 for fast
	//inst->sercom->BAUD.reg = (uint8_t) SPI_BAUD_RATE; //SERCOM_SPI_BAUD_BAUD(1000);

    // Enable receive complete (RXC) interrupt
    //inst->sercom->INTENSET.bit.RXC = 1;

    // NVIC interrupt enable
	//NVIC_EnableIRQ(SERCOM4_2_IRQn);

	/* Configure pins for the correct function. */
	gpio_init_pin(inst->mosi, GPIO_DIR_OUT, inst->mosi_alt);
	gpio_init_pin(inst->miso, GPIO_DIR_IN, inst->miso_alt);
	gpio_init_pin(inst->sck, GPIO_DIR_OUT, inst->sck_alt);
	gpio_init_pin(inst->cs, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	
	gpio_set_pin(inst->cs);

	/* Finally, enable it! */
	inst->sercom->CTRLB.bit.RXEN = 1;
	inst->sercom->CTRLA.bit.ENABLE = 1;
	while (inst->sercom->SYNCBUSY.bit.ENABLE) {};
}

void spi_change_mode(const SPIConfig* inst) {
    // Disable SPI
    inst->sercom->CTRLA.bit.ENABLE = 0;
    while (inst->sercom->SYNCBUSY.bit.ENABLE);

    // Modify CPOL and CPHA
    inst->sercom->CTRLA.bit.CPOL = inst->polarity; // or 0
    inst->sercom->CTRLA.bit.CPHA = inst->phase; // or 0
    
    // Enable SPI again
    inst->sercom->CTRLA.bit.ENABLE = 1;
    while (inst->sercom->SYNCBUSY.bit.ENABLE);
}

uint16_t spi_write_read16(const SPIConfig* inst, uint16_t data) {
	uint8_t byte0 = data & 0xFF;
	uint8_t byte1 = (data & 0xFF00) >> 8;
	uint8_t read1 = 0;
	uint8_t read0 = 0;
	
	// Bring nCS low
	gpio_clear_pin(inst->cs);
    delay(CS_DELAY);
    //for (int i = 0; i < 0xFF; i++);
	
	// Transmit first byte
	while (!inst->sercom->INTFLAG.bit.DRE) {}
	inst->sercom->DATA.bit.DATA = byte1;
	while (!inst->sercom->INTFLAG.bit.RXC) {}
	read1 = inst->sercom->DATA.reg & 0xFF;
	
	// Transmit zeroeth byte
	while (!inst->sercom->INTFLAG.bit.DRE) {}
	inst->sercom->DATA.bit.DATA = byte0;
	while (!inst->sercom->INTFLAG.bit.RXC) {}
	read0 = inst->sercom->DATA.reg & 0xFF;
	
	// Bring nCS high
    //for (int i = 0; i < 0xFF; i++);
    delay(CS_DELAY);
	gpio_set_pin(inst->cs);
	uint16_t result = (read1 << 8) | read0;
	return result;
}

uint32_t spi_write_read24(const SPIConfig* inst, uint32_t data) {
	uint8_t byte0 = data & 0xFF;
	uint8_t byte1 = (data >> 8) & 0xFF;
	uint8_t byte2 = (data >> 16) & 0xFF;
	uint8_t read2 = 0;
	uint8_t read1 = 0;
	uint8_t read0 = 0;

	// Bring nCS low
	gpio_clear_pin(inst->cs);
    delay(CS_DELAY);

	// Transmit byte 2 (MSB)
	while (!inst->sercom->INTFLAG.bit.DRE) {}
	inst->sercom->DATA.bit.DATA = byte2;
	while (!inst->sercom->INTFLAG.bit.RXC) {}
	read2 = inst->sercom->DATA.reg & 0xFF;

	// Transmit byte 1
	while (!inst->sercom->INTFLAG.bit.DRE) {}
	inst->sercom->DATA.bit.DATA = byte1;
	while (!inst->sercom->INTFLAG.bit.RXC) {}
	read1 = inst->sercom->DATA.reg & 0xFF;

	// Transmit byte 0 (LSB)
	while (!inst->sercom->INTFLAG.bit.DRE) {}
	inst->sercom->DATA.bit.DATA = byte0;
	while (!inst->sercom->INTFLAG.bit.RXC) {}
	read0 = inst->sercom->DATA.reg & 0xFF;

	// Bring nCS high
    delay(CS_DELAY);
	gpio_set_pin(inst->cs);
	uint32_t result = (read2 << 16) | (read1 << 8) | read0;
	return result;
}

int32_t spi_write_read16_checksum(const SPIConfig* inst, uint16_t data) {
	uint8_t byte0 = data & 0xFF;
	uint8_t byte1 = (data >> 8) & 0xFF;
	uint8_t checksum = (byte1 + byte0 + 0x55) & 0xFF;
	uint32_t data24 = ((uint32_t)data << 8) | checksum;
	uint32_t result = spi_write_read24(inst, data24);
	uint8_t rx_byte1 = (result >> 16) & 0xFF;
	uint8_t rx_byte0 = (result >> 8) & 0xFF;
	uint8_t rx_checksum = result & 0xFF;
	uint8_t expected = (rx_byte1 + rx_byte0 + 0x55) & 0xFF;
	if (rx_checksum != expected) {
		return -1;
	}
	return (rx_byte1 << 8) | rx_byte0;
}

