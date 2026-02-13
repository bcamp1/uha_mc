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

#define RS422_TX_PIN		PIN_PA04
#define RS422_RX_PIN		PIN_PA05
#define RS422_TX_PAD		(0)
#define RS422_RX_PAD		(1)
#define RS422_SERCOM		SERCOM0
#define RS422_BAUD_9600	(0xD8A0)

static char read_character = 0;

void rs422_init(void) {
	// Init ports
	gpio_init_pin(RS422_TX_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_D_SERCOM_ALT);
	gpio_init_pin(RS422_RX_PIN, GPIO_DIR_IN, GPIO_ALTERNATE_D_SERCOM_ALT);
	
	// Init clock
	wntr_sercom_init_clock(RS422_SERCOM, GCLK_PCHCTRL_GEN_GCLK4);
	
	// Enable clock TODO
	//PM->APBCMASK.bit.SERCOM5_ = 1; // Enable clock source
	//GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID_SERCOM5_CORE | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_CLKEN;
	//while (GCLK->STATUS.bit.SYNCBUSY) {} // Wait for busy
	
	RS422_SERCOM->USART.CTRLA.bit.MODE = 1; // Enable internal clock
	RS422_SERCOM->USART.CTRLA.bit.TXPO = 0x0; // Pad 0
	RS422_SERCOM->USART.CTRLA.bit.RXPO = 0x1; // Pad 1
	RS422_SERCOM->USART.CTRLA.bit.DORD = 1; // Order

	RS422_SERCOM->USART.BAUD.reg = RS422_BAUD_9600;
	
	// Enable recieve interrupts
	//RS422_SERCOM->USART.INTENSET.bit.RXC = 1;
	//NVIC_EnableIRQ(SERCOM0_2_IRQn);
	
	RS422_SERCOM->USART.CTRLB.bit.CHSIZE = 0;
	RS422_SERCOM->USART.CTRLB.bit.TXEN = 1;
	RS422_SERCOM->USART.CTRLB.bit.RXEN = 1;
	while (RS422_SERCOM->USART.SYNCBUSY.bit.CTRLB) {} // Wait for busy
	
	RS422_SERCOM->USART.CTRLA.bit.ENABLE = 1;
	while (RS422_SERCOM->USART.SYNCBUSY.bit.ENABLE) {} // Wait for busy
}

void rs422_put(char ch) {
	delay(0x1F);
	//for (int i = 0; i < 0x8FF; i++);
	RS422_SERCOM->USART.DATA.reg = ch; // Send data
	while (!RS422_SERCOM->USART.INTFLAG.bit.DRE) {} // Wait for empty
	if (ch == '\n') {
		rs422_put('\r');
	}
}

char rs422_get() {
	if (RS422_SERCOM->USART.INTFLAG.bit.RXC) {
		return (RS422_SERCOM->USART.DATA.reg & 0xff);
	}
	return 0;
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
