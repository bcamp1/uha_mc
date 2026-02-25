/*
 * uart.c
 *
 * Created: 7/29/2024 3:41:25 PM
 *  Author: brans
 */

#include "rs422.h"
#include "../periphs/gpio.h"
#include "samd51j20a.h"
#include "../periphs/sercom.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "../drivers/delay.h"
#include "../sched.h"

#define RS422_TX_PIN		PIN_PA04
#define RS422_RX_PIN		PIN_PA05
#define RS422_TX_PAD		(0)
#define RS422_RX_PAD		(1)
#define RS422_SERCOM		SERCOM0
#define RS422_BAUD_9600	(0xD8A0)

#define RS422_BUF_SIZE 64

// RX ring buffer
static volatile uint8_t rx_buf[RS422_BUF_SIZE];
static volatile uint16_t rx_head = 0;  // ISR writes here
static volatile uint16_t rx_tail = 0;  // rs422_get reads here

// TX ring buffer
static volatile uint8_t tx_buf[RS422_BUF_SIZE];
static volatile uint16_t tx_head = 0;  // rs422_put writes here
static volatile uint16_t tx_tail = 0;  // ISR reads here

void rs422_init(void) {
	// Init ports
	gpio_init_pin(RS422_TX_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(RS422_RX_PIN, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);

	// Init clock
	wntr_sercom_init_clock(RS422_SERCOM, GCLK_PCHCTRL_GEN_GCLK4);

	RS422_SERCOM->USART.CTRLA.bit.MODE = 1; // Enable internal clock
	RS422_SERCOM->USART.CTRLA.bit.TXPO = 0x0; // Pad 0
	RS422_SERCOM->USART.CTRLA.bit.RXPO = 0x1; // Pad 1
	RS422_SERCOM->USART.CTRLA.bit.DORD = 1; // Order

	RS422_SERCOM->USART.BAUD.reg = RS422_BAUD_9600;

	// Enable RXC interrupt (fires when byte received)
	RS422_SERCOM->USART.INTENSET.bit.RXC = 1;

	// DRE interrupt is NOT enabled here — enabled on-demand by rs422_put()

	// SERCOM0_0 = DRE, SERCOM0_2 = RXC
	NVIC_SetPriority(SERCOM0_0_IRQn, PRIO_RS422);
	NVIC_SetPriority(SERCOM0_2_IRQn, PRIO_RS422);
	NVIC_EnableIRQ(SERCOM0_0_IRQn);
	NVIC_EnableIRQ(SERCOM0_2_IRQn);

	RS422_SERCOM->USART.CTRLB.bit.CHSIZE = 0;
	RS422_SERCOM->USART.CTRLB.bit.TXEN = 1;
	RS422_SERCOM->USART.CTRLB.bit.RXEN = 1;
	while (RS422_SERCOM->USART.SYNCBUSY.bit.CTRLB) {} // Wait for busy

	RS422_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	while (RS422_SERCOM->USART.SYNCBUSY.bit.ENABLE) {} // Wait for busy
}

// RX ISR (SERCOM0_2 = RXC)
void SERCOM0_2_Handler(void) {
	if (RS422_SERCOM->USART.INTFLAG.bit.RXC) {
		uint8_t data = RS422_SERCOM->USART.DATA.reg & 0xFF;
		uint16_t next = (rx_head + 1) % RS422_BUF_SIZE;
		if (next != rx_tail) {  // Drop byte if buffer full
			rx_buf[rx_head] = data;
			rx_head = next;
		}
	}
}

// TX ISR (SERCOM0_0 = DRE)
void SERCOM0_0_Handler(void) {
	if (RS422_SERCOM->USART.INTFLAG.bit.DRE) {
		if (tx_head != tx_tail) {
			RS422_SERCOM->USART.DATA.reg = tx_buf[tx_tail];
			tx_tail = (tx_tail + 1) % RS422_BUF_SIZE;
		} else {
			// Buffer empty — disable DRE interrupt until next rs422_put()
			RS422_SERCOM->USART.INTENCLR.bit.DRE = 1;
		}
	}
}

static void tx_enqueue(uint8_t ch) {
	uint16_t next = (tx_head + 1) % RS422_BUF_SIZE;
	while (next == tx_tail) {}  // Spin if buffer full (back-pressure)
	tx_buf[tx_head] = ch;
	tx_head = next;
	RS422_SERCOM->USART.INTENSET.bit.DRE = 1;  // Kick DRE ISR
}

void rs422_put(char ch) {
    /*
	tx_enqueue((uint8_t)ch);
	if (ch == '\n') {
		tx_enqueue('\r');
	}
    */
}

int16_t rs422_get(void) {
	if (rx_head == rx_tail) {
		return RS422_EMPTY;
	}
	uint8_t data = rx_buf[rx_tail];
	rx_tail = (rx_tail + 1) % RS422_BUF_SIZE;
	return (int16_t)data;
}

void rs422_print(char* str) {
	char* ch = str;
	while (*ch != '\0') {
		rs422_put(*ch);
		ch++;
	}
}

void rs422_println(char* str) {
	rs422_print(str);
	rs422_put('\n');
}


#define MAX_FLOAT_STR_LEN 32  // Define max length for the float string (e.g., 32 chars)
#define FLOAT_PRECISION (4)

void rs422_print_float(float num) {
	if (num < 0) {
		rs422_put('-');
		num = -num;
	} else {
        rs422_put('+');
    }
	static char buffer[MAX_FLOAT_STR_LEN];  // Static to keep the buffer valid after returning
	int integer_part = (int)num;  // Get the integer part
	float fractional_part = num - integer_part;  // Get the fractional part

	// Convert integer part to string
	int i = 0;
	if (integer_part == 0) {
		buffer[i++] = '0';  // Handle zero integer part
		} else {
		char int_buffer[10];  // Temporary buffer for integer part
		int idx = 0;
		while (integer_part > 0) {
			int_buffer[idx++] = (integer_part % 10) + '0';
			integer_part /= 10;
		}

		// Reverse integer part into buffer
		for (int j = idx - 1; j >= 0; j--) {
			buffer[i++] = int_buffer[j];
		}
	}

	// Add decimal point
	buffer[i++] = '.';

	// Convert fractional part to string (limited to 6 decimal places)
	int frac_digits = FLOAT_PRECISION;
	while (frac_digits-- > 0) {
		fractional_part *= 10;
		int digit = (int)fractional_part;
		buffer[i++] = digit + '0';
		fractional_part -= digit;
	}

	buffer[i] = '\0';  // Null-terminate the string
	rs422_print(buffer);
}

void rs422_println_float(float num) {
	rs422_print_float(num);
	rs422_put('\n');
}

#define MAX_INT_STR_LEN 50
void rs422_print_int_base(int num, int base) {
	static char buffer[MAX_INT_STR_LEN];
	//buffer[0] = 0;
	itoa(num, buffer, base);
	rs422_print(buffer);
}

void rs422_println_int_base(int num, int base) {
	rs422_print_int_base(num, base);
	rs422_put('\n');
}

void rs422_print_int(int num) {
	rs422_print_int_base(num, 10);
}

void rs422_println_int(int num) {
	rs422_println_int_base(num, 10);
}

void rs422_send_float(float num) {
	uint32_t raw = 0;
	memcpy(&raw, &num, 4);
	char byte0 = (raw >> 0)  & 0xFF;  // Least significant byte
    char byte1 = (raw >> 8)  & 0xFF;
    char byte2 = (raw >> 16) & 0xFF;
    char byte3 = (raw >> 24) & 0xFF;  // Most significant byte
	rs422_put(byte3);
	rs422_put(byte2);
	rs422_put(byte1);
	rs422_put(byte0);
}

void rs422_send_float_arr(float* data, int len) {
    for (int i = 0; i < len; i++) {
        rs422_send_float(data[i]);
        delay(0x3FF);
    }
    rs422_send_float(INFINITY);
    delay(0x3FF);
}

void rs422_print_float_arr(float* data, int len) {
   // rs422_put('[');
    for (int i = 0; i < len; i++) {
        rs422_print_float(data[i]);
        if (i < len-1) {
            rs422_put(',');
            //rs422_put(' ');
        }
    }
   // rs422_put(']');
}

void rs422_println_float_arr(float* data, int len) {
    rs422_print_float_arr(data, len);
    rs422_put('\n');
}
