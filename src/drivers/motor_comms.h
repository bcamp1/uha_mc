#pragma once
#include <stdint.h>
#include <stdbool.h>

void motor_comms_init();
void motor_comms_send_bytes(uint8_t addr, const uint8_t *data, uint8_t length);
void motor_comms_send_cmd(uint8_t addr, uint8_t cmd);
void motor_comms_send_float(uint8_t addr, const uint8_t command, float data);
float motor_comms_data_to_float(uint8_t* data);
bool motor_comms_get_data(uint8_t* data, uint8_t* data_len, uint8_t buf_size);

