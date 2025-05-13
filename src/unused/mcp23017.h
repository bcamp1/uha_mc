/*
 * mcp23017.h
 *
 * Created: 8/10/2024 1:18:44 AM
 *  Author: brans
 */ 


#ifndef MCP23017_H_
#define MCP23017_H_

#include <stdint.h>
#include <stdbool.h>


// MCP23017 Registers
#define MCP_IODIRA		(0x00)
#define MCP_IODIRB		(0x01)
#define MCP_IPOLA		(0x02)
#define MCP_IPOLB		(0x03)
#define MCP_GPINTENA	(0x04)
#define MCP_GPINTENB	(0x05)
#define MCP_DEFVALA		(0x06)
#define MCP_DEFVALB		(0x07)
#define MCP_INTCONA		(0x08)
#define MCP_INTCONB		(0x09)
#define MCP_IOCON		(0x0A)
#define MCP_GPPUA		(0x0C)
#define MCP_GPPUB		(0x0D)
#define MCP_INTFA		(0x0E)
#define MCP_INTFB		(0x0F)
#define MCP_INTCAPA		(0x10)
#define MCP_INTCAPB		(0x11)
#define MCP_GPIOA		(0x12)
#define MCP_GPIOB		(0x13)
#define MCP_OLATA		(0x14)
#define MCP_OLATB		(0x15)

// IOCON bits
#define MCP_IOCON_BANK		(0b10000000)
#define MCP_IOCON_MIRROR	(0b01000000)
#define MCP_IOCON_SEQOP		(0b00100000)
#define MCP_IOCON_DISSLW	(0b00010000)
#define MCP_IOCON_ODR		(0b00000100)
#define MCP_IOCON_INTPOL	(0b00000010)

void mcp23017_init(void);
int mcp23017_write_reg(uint8_t addr, uint8_t reg, uint8_t value);
int mcp23017_read_reg(uint8_t addr, uint8_t reg, uint8_t* value);
void mcp23017_print_info(uint8_t addr);
int mcp23017_swrst(uint8_t addr);

#endif /* MCP23017_H_ */