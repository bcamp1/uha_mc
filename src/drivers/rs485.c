/*
 * rs485.c
 *
 * Created: 7/29/2024 3:41:25 PM
 *  Author: brans
 */

#include "rs485.h"
#include "../periphs/gpio.h"
#include "samd51j20a.h"
#include "../periphs/sercom.h"
#include <string.h>
#include "../drivers/delay.h"

#define RS485_TX_PIN		PIN_PA12
#define RS485_RX_PIN		PIN_PA13
#define RS485_TX_PAD		(0)
#define RS485_RX_PAD		(1)
#define RS485_SERCOM		SERCOM2
#define RS485_TXEN_PIN		PIN_PA14
#define RS485_BAUD  	    (0x9000)

void rs485_init(void) {
	// Init ports
	gpio_init_pin(RS485_TX_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_C_SERCOM);
	gpio_init_pin(RS485_RX_PIN, GPIO_DIR_IN, GPIO_ALTERNATE_C_SERCOM);
	gpio_init_pin(RS485_TXEN_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_clear_pin(RS485_TXEN_PIN); // Default to listening

	// Init clock
	wntr_sercom_init_clock(RS485_SERCOM, GCLK_PCHCTRL_GEN_GCLK4);

	RS485_SERCOM->USART.CTRLA.bit.MODE = 1; // Enable internal clock
	RS485_SERCOM->USART.CTRLA.bit.TXPO = 0x0; // Pad 0
	RS485_SERCOM->USART.CTRLA.bit.RXPO = 0x1; // Pad 1
	RS485_SERCOM->USART.CTRLA.bit.DORD = 1; // Order

	RS485_SERCOM->USART.BAUD.reg = RS485_BAUD;

	RS485_SERCOM->USART.CTRLB.bit.CHSIZE = 0;
	RS485_SERCOM->USART.CTRLB.bit.TXEN = 1;
	RS485_SERCOM->USART.CTRLB.bit.RXEN = 1;
	while (RS485_SERCOM->USART.SYNCBUSY.bit.CTRLB) {} // Wait for busy

	RS485_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	while (RS485_SERCOM->USART.SYNCBUSY.bit.ENABLE) {} // Wait for busy
}

static void rs485_tx_begin(void) {
	gpio_set_pin(RS485_TXEN_PIN);
	delay(0x4E); // Settle before transmitting
}

static void rs485_tx_end(void) {
	while (!RS485_SERCOM->USART.INTFLAG.bit.TXC) {} // Wait for last byte to shift out
	delay(0x4E); // Settle before releasing bus
	gpio_clear_pin(RS485_TXEN_PIN);
}

static void rs485_put_byte(uint8_t byte) {
	RS485_SERCOM->USART.DATA.reg = byte;
	while (!RS485_SERCOM->USART.INTFLAG.bit.DRE) {}
}

int16_t rs485_get(void) {
	if (RS485_SERCOM->USART.INTFLAG.bit.RXC) {
		return (RS485_SERCOM->USART.DATA.reg & 0xFF);
	}
	return RS485_EMPTY;
}

void rs485_send_byte(uint8_t byte) {
	rs485_tx_begin();
	rs485_put_byte(byte);
	rs485_tx_end();
}

void rs485_send_uint(uint32_t val) {
	rs485_tx_begin();
	rs485_put_byte((val >> 24) & 0xFF);
	rs485_put_byte((val >> 16) & 0xFF);
	rs485_put_byte((val >> 8)  & 0xFF);
	rs485_put_byte((val >> 0)  & 0xFF);
	rs485_tx_end();
}

void rs485_send_float(float num) {
	uint32_t raw = 0;
	memcpy(&raw, &num, 4);
	rs485_send_uint(raw);
}

void rs485_send_float_arr(float* data, int len) {
	rs485_tx_begin();
	for (int i = 0; i < len; i++) {
		uint32_t raw = 0;
		memcpy(&raw, &data[i], 4);
		rs485_put_byte((raw >> 24) & 0xFF);
		rs485_put_byte((raw >> 16) & 0xFF);
		rs485_put_byte((raw >> 8)  & 0xFF);
		rs485_put_byte((raw >> 0)  & 0xFF);
	}
	rs485_tx_end();
}
