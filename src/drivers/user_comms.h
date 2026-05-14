#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_comms.h"   // RXError (RX_ERR_OK, RX_ERR_NO_DATA, ...)

#define COMMS_MAX_DATA 32

#define COMMS_CMD_ACTION_STOP       0x01
#define COMMS_CMD_ACTION_PLAYBACK   0x02
#define COMMS_CMD_ACTION_REWIND     0x03
#define COMMS_CMD_ACTION_FF         0x04
#define COMMS_CMD_ACTION_MEM        0x05
#define COMMS_CMD_ACTION_RTZ        0x06
#define COMMS_CMD_TRANSMIT_TAPE_POS 0x08
#define COMMS_CMD_TRANSMIT_TAPE_SPD 0x09

typedef struct {
    RXError err;                       // RX_ERR_OK if a frame was latched, RX_ERR_NO_DATA otherwise
    uint8_t data[COMMS_MAX_DATA];
    uint8_t data_len;
} CommsRxResult;

typedef void (*CommsMessageReadyCb)(void);

void  comms_init(void);
void  comms_send_bytes(const uint8_t *data, uint8_t length);
void  comms_send_float(uint8_t command, float data);
float comms_data_to_float(const uint8_t* data);

// Returns the latched frame (consuming clears it), or { .err = RX_ERR_NO_DATA }.
CommsRxResult comms_get_data(void);

// Invoked from the RXC ISR right after a checksum-valid frame is latched.
// Keep work minimal — this is ISR context. comms_get_data() inside the
// callback returns the freshly-latched frame.
void comms_register_message_ready_cb(CommsMessageReadyCb cb);
