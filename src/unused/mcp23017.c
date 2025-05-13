/*
 * mcp23017.c
 *
 * Created: 8/10/2024 1:18:19 AM
 *  Author: brans
 */ 

#include "samd51j20a.h"
#include <stdio.h>

#include "mcp23017.h"
#include "../periphs/gpio.h"
#include "../periphs/uart.h"
#include "../periphs/i2c.h"

void mcp23017_init() {
	i2c_init();
}

int mcp23017_write_reg(uint8_t addr, uint8_t reg, uint8_t value) {
	// Wait for bus state idle
	i2c_wait_busy();
	
	// Send address (OP)
	if (i2c_send_addr_write(addr) != 0) {
		return 1;
	}
	
	// Send address data (ADDR)
	if (i2c_send_data(reg) != 0) {
		return 1;
	}
	
	// Send data to write
	if (i2c_send_data(value) != 0) {
		return 1;
	}
	
	i2c_send_cmd(I2C_CMD_NACK, I2C_CMD_STOP);
	return 0;
}

int mcp23017_read_reg(uint8_t addr, uint8_t reg, uint8_t* value) {
	// Wait for bus state idle
	i2c_wait_busy();
	
	// Send address (OP)
	if (i2c_send_addr_write(addr) != 0) {
		return 1;
	}
	
	// Send address data (ADDR)
	if (i2c_send_data(reg) != 0) {
		return 2;
	}
	
	i2c_send_cmd(I2C_CMD_ACK, I2C_CMD_RS);
	
	// Repeated start, send OP + R
	if (i2c_send_addr_read(addr) != 0) {
		return 3;
	}
	
	*value = I2C_SERCOM->I2CM.DATA.reg;
	i2c_syncbusy();
	//*value = MCP_SERCOM->I2CM.DATA.reg;
	//i2c_syncbusy();
	
	i2c_send_cmd(I2C_CMD_NACK, I2C_CMD_STOP);
	i2c_syncbusy();
	return 0;
}



void mcp23017_print_info(uint8_t addr) {
	//uart_print("\n\r");
	uint8_t value = 0;
	char buf[40];
	for (uint8_t i = 0; i < 0x16; i++) {
		if (i != 0xB) {
			mcp23017_read_reg(addr, i, &value);
			sprintf(buf, "0x%02X: %02X\n\r", i, value);
			uart_print(buf);
		}
	}
	uart_print("\n\r");
}

int mcp23017_swrst(uint8_t addr) {
	for (uint8_t reg = 0; reg < 0x16; reg++) {
		uint8_t reset_value = 0x00;
		if (reg < 0x04) {
			reset_value = 0xFF;
		}
		if (mcp23017_write_reg(addr, reg, reset_value) != 0) return 1;
	}
	return 0;
}

