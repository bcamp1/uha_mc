/*
 * i2c.c
 *
 * Created: 8/17/2024 12:06:31 PM
 *  Author: brans
 */ 

#include "samd51j20a.h"
#include <stdio.h>

#include "i2c.h"
#include "gpio.h"

void i2c_init(void) {
	// Enable the APBC clock for SERCOM0
	/*
	PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;

	// Enable GCLK for SERCOM0
	GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID(SERCOM0_GCLK_ID_CORE) |
	GCLK_CLKCTRL_GEN(0) | // Use GCLK0 (or the generator you're using)
	GCLK_CLKCTRL_CLKEN;
	while (GCLK->STATUS.bit.SYNCBUSY); // Wait for synchronization
*/
	// Initialize pins
	gpio_init_pin(I2C_SDA, GPIO_DIR_OUT, GPIO_ALTERNATE_C_SERCOM);
	gpio_init_pin(I2C_SCL, GPIO_DIR_OUT, GPIO_ALTERNATE_C_SERCOM);
	
	I2C_SERCOM->I2CM.CTRLA.bit.SWRST = 1;
	i2c_syncbusy();
	
	
	// Select host mode
	I2C_SERCOM->I2CM.CTRLA.bit.MODE = 0x5;	// Host mode
	
	// Enable smart mode
	//MCP_SERCOM->I2CM.CTRLB.bit.SMEN = 1;
	i2c_syncbusy();
	
	// Select baud rate
	I2C_SERCOM->I2CM.BAUD.reg = 0xFFF0; // Desired baud rate
	
	// Enable the peripheral
	I2C_SERCOM->I2CM.CTRLA.bit.ENABLE = 1;
	
	// Wait for synchronization
	i2c_syncbusy();
	

	// Force the bus state to idle
	SERCOM0->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);
	i2c_syncbusy();
	
	
	// Wait for bus state idle
	i2c_wait_busy();
	
	for (uint16_t i = 0; i < 0xFF; i++) {}
	//MCP_SERCOM->I2CM.ADDR.bit.ADDR = 0x4;
}

int i2c_wait_MB_timeout(uint32_t cycles) {
	for (uint32_t i = 0; i < cycles; i++) {
		if (I2C_SERCOM->I2CM.INTFLAG.bit.MB == 1) {
			return 0;
		}
	}
	return 1;
}

void i2c_wait_busy() {
	while (I2C_SERCOM->I2CM.STATUS.bit.BUSSTATE == I2C_BUS_BUSY) {
		// Do nothing
	}
}

void i2c_syncbusy() {
	while ((I2C_SERCOM->I2CM.SYNCBUSY.reg & 0b111) != 0) {
		// Do nothing
	}
}

void i2c_send_cmd(bool ack, uint8_t cmd) {
	I2C_SERCOM->I2CM.CTRLB.bit.ACKACT = ack;
	I2C_SERCOM->I2CM.CTRLB.bit.CMD = cmd;
	i2c_syncbusy();
}

int i2c_send_data(uint8_t data) {
	I2C_SERCOM->I2CM.DATA.reg = data;
	i2c_syncbusy();
	
	//while (MCP_SERCOM->I2CM.INTFLAG.bit.MB == 0);
	if (i2c_wait_MB_timeout(0xFFFF) != 0) return 1;
	
	if (I2C_SERCOM->I2CM.STATUS.bit.RXNACK == 1) {
		i2c_send_cmd(I2C_CMD_NACK, I2C_CMD_STOP);
		return 1; // Client responded with NACK
	}
	
	if (I2C_SERCOM->I2CM.INTFLAG.bit.ERROR) {
		//MCP_SERCOM->I2CM.INTFLAG.bit.ERROR = 0;
		SERCOM0->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);
		while (SERCOM0->I2CM.SYNCBUSY.bit.SYSOP);
		return 1;
	}
	return 0;
}

int i2c_send_addr_write(uint8_t addr) {
	i2c_wait_busy();
	uint8_t joined_address = (addr << 1);
	I2C_SERCOM->I2CM.ADDR.bit.ADDR = joined_address;
	i2c_syncbusy();
	
	// while (MCP_SERCOM->I2CM.INTFLAG.bit.MB == 0);
	if (i2c_wait_MB_timeout(0xFFFF) != 0) return 1;
	
	if (I2C_SERCOM->I2CM.STATUS.bit.RXNACK == 1) {
		i2c_send_cmd(I2C_CMD_NACK, I2C_CMD_STOP);
		return 12; // Client responded with NACK
	}
	
	if (I2C_SERCOM->I2CM.INTFLAG.bit.ERROR) {
		//MCP_SERCOM->I2CM.INTFLAG.bit.ERROR = 0;
		SERCOM0->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);
		while (SERCOM0->I2CM.SYNCBUSY.bit.SYSOP);
		return 13;
	}
	return 0;
}

int i2c_send_addr_read(uint8_t addr) {
	i2c_wait_busy();
	uint8_t joined_address = (addr << 1) | I2C_R;
	I2C_SERCOM->I2CM.ADDR.bit.ADDR = joined_address;
	i2c_syncbusy();
	
	while (I2C_SERCOM->I2CM.INTFLAG.bit.SB == 0);
	if (I2C_SERCOM->I2CM.STATUS.bit.RXNACK == 1) {
		i2c_send_cmd(I2C_CMD_NACK, I2C_CMD_STOP);
		return 1; // Client responded with NACK
	}
	
	if (I2C_SERCOM->I2CM.INTFLAG.bit.ERROR) {
		//MCP_SERCOM->I2CM.INTFLAG.bit.ERROR = 0;
		SERCOM0->I2CM.STATUS.reg |= SERCOM_I2CM_STATUS_BUSSTATE(1);
		while (SERCOM0->I2CM.SYNCBUSY.bit.SYSOP);
		return 1;
	}
	return 0;
}