/*
 * uart.h
 *
 * Created: 7/29/2024 3:41:36 PM
 *  Author: brans
 */ 

#pragma once

void rs422_init(void);

// Input
char rs422_get();

// Strings
void rs422_put(char ch);
void rs422_print(char* str);
void rs422_println(char* str);

// Integers
void rs422_print_int_base(int num, int base);
void rs422_println_int_base(int num, int base);
void rs422_print_int(int num);
void rs422_println_int(int num);

// Floats
void rs422_print_float(float num);
void rs422_println_float(float num);
void rs422_print_float_arr(float* data, int len);
void rs422_println_float_arr(float* data, int len);

// Send binary data
void rs422_send_float(float num);
void rs422_send_float_arr(float* data, int len);

