#pragma once
#include <stdint.h>
#include <stdbool.h>

#define COMMS_CMD_DEBUG             0x00
#define COMMS_CMD_DEBUG_FLOAT       0x07
#define COMMS_CMD_ACTION_STOP       0x01
#define COMMS_CMD_ACTION_PLAYBACK   0x02
#define COMMS_CMD_ACTION_REWIND     0x03
#define COMMS_CMD_ACTION_FF         0x04
#define COMMS_CMD_ACTION_MEM        0x05
#define COMMS_CMD_ACTION_RTZ        0x06
#define COMMS_CMD_TRANSMIT_TAPE_POS 0x08
#define COMMS_CMD_TRANSMIT_TAPE_SPD 0x09

void comms_init();
void comms_send_bytes(const uint8_t *data, uint8_t length);
void comms_send_float(const uint8_t command, float data);
float comms_data_to_float(uint8_t* data);
bool comms_get_data(uint8_t* data, uint8_t* data_len, uint8_t buf_size);
void comms_print_debug(const char* debug_str);

