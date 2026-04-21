/*
 * rs485.h
 *
 * Created: 7/29/2024 3:41:36 PM
 *  Author: brans
 */

#pragma once

#include <stdint.h>

#define RS485_EMPTY (-1)

void rs485_init(void);

int16_t rs485_get(void);

void rs485_send_byte(uint8_t byte);
void rs485_send_bytes(const uint8_t* data, uint16_t len);
void rs485_send_uint(uint32_t val);
void rs485_send_float(float num);
void rs485_send_float_arr(float* data, int len);
