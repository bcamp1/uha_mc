/*
 * uart.h
 *
 * Created: 7/29/2024 3:41:36 PM
 *  Author: brans
 */ 

#ifndef UART_H_
#define UART_H_


void uart_init(void);

// Input
char uart_get();

// Strings
void uart_put(char ch);
void uart_print(char* str);
void uart_println(char* str);

// Integers
void uart_print_int_base(int num, int base);
void uart_println_int_base(int num, int base);
void uart_print_int(int num);
void uart_println_int(int num);

// Floats
void uart_print_float(float num);
void uart_println_float(float num);

// Send binary data
void uart_send_float(float num);
void uart_send_float_arr(float* data, int len);

#endif /* UART_H_ */
