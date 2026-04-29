#pragma once
#include <stdint.h>
#include <stdbool.h>

typedef enum {
    RX_ERR_OK,
    RX_ERR_TIMEOUT,
    RX_ERR_WRONG_ADDR,
    RX_ERR_BUF_OVF,
    RX_ERR_WRONG_CHECKSUM,
    RX_ERR_NO_DATA,
} RXError;

// Available Addresses
#define MOTOR_COMMS_ADDR_BROADCAST (0x55)
#define MOTOR_COMMS_ADDR_TAKEUP    (0b100)
#define MOTOR_COMMS_ADDR_SUPPLY    (0b010)
#define MOTOR_COMMS_ADDR_CAPSTAN   (0b001)

// Available Commands
#define MOTOR_COMMS_CMD_DISABLE (0x0)
#define MOTOR_COMMS_CMD_ENABLE (0x1)
#define MOTOR_COMMS_CMD_FAULT_STATUS (0x2)
#define MOTOR_COMMS_CMD_CALIB_ENCODER (0x3)

void motor_comms_init();
void motor_comms_send_bytes(uint8_t addr, const uint8_t *data, uint8_t length);
void motor_comms_send_cmd(uint8_t addr, uint8_t cmd);

void motor_comms_broadcast_bytes(const uint8_t *data, uint8_t length);
void motor_comms_broadcast_cmd(uint8_t cmd);

RXError motor_comms_get_data(uint8_t* addr, uint8_t* data, uint8_t* data_len, uint8_t buf_size);
RXError motor_comms_read(uint8_t addr, uint8_t cmd, uint8_t* data, uint8_t* data_len, uint8_t buf_size);

void motor_comms_print_error(RXError err);
void motor_comms_println_error(RXError err);

