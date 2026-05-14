#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "motor_comms.h"   // RXError (RX_ERR_OK, RX_ERR_NO_DATA, ...)

#define COMMS_MAX_DATA 32

typedef struct {
    RXError err;                       // RX_ERR_OK if a frame was latched, RX_ERR_NO_DATA otherwise
    uint8_t data[COMMS_MAX_DATA];
    uint8_t data_len;
} CommsRxResult;

typedef void (*CommsMessageReadyCb)(void);

void  comms_init(void);
void  comms_send_bytes(const uint8_t *data, uint8_t length);
void  comms_send_float(uint8_t command, float data);
void  comms_send_float2(uint8_t command, float data1, float data2);
float comms_data_to_float(const uint8_t* data);
void comms_send_byte(uint8_t data);

// Returns the latched frame (consuming clears it), or { .err = RX_ERR_NO_DATA }.
CommsRxResult comms_get_data(void);

// Invoked from the RXC ISR right after a checksum-valid frame is latched.
// Keep work minimal — this is ISR context. comms_get_data() inside the
// callback returns the freshly-latched frame.
void comms_register_message_ready_cb(CommsMessageReadyCb cb);
