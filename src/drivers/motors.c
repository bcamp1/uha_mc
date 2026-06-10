#include "motors.h"
#include "motor_comms.h"
#include "faults.h"
#include "delay.h"
#include <stddef.h>
#include <stdint.h>

// +1.0f -> INT16_MAX, -1.0f -> INT16_MIN. Asymmetric scaling because the
// negative range has one more representable value than the positive range.
static int16_t torque_float_to_int16(float torque) {
    if (torque >= 1.0f)  return INT16_MAX;
    if (torque <= -1.0f) return INT16_MIN;
    if (torque >= 0.0f)  return (int16_t)(torque * 32767.0f);
    return (int16_t)(torque * 32768.0f);
}

static RXError confirm_command(uint8_t address, uint8_t command) {
    uint8_t data_len = 0;
    uint8_t buf[5];
    RXError err = motor_comms_read(address, command, buf, &data_len, 5);
    if (err != RX_ERR_OK) return err;
    if (data_len == 0) return RX_ERR_WRONG_RESPONSE_LENGTH;
    if (buf[0] != command) return RX_ERR_WRONG_RESPONSE;
    return RX_ERR_OK;
}

void motors_init() {
    motor_comms_init();
}

void motors_enable_all() {
    motor_comms_broadcast_cmd(MOTOR_COMMS_CMD_ENABLE);
}

void motors_disable_all() {
    motor_comms_broadcast_cmd(MOTOR_COMMS_CMD_DISABLE);
}

void motors_set_reel_torques(float takeup_torque, float supply_torque) {
    motors_set_torque(MOTOR_COMMS_ADDR_TAKEUP, takeup_torque);
    delay(0xFF);
    motors_set_torque(MOTOR_COMMS_ADDR_SUPPLY, supply_torque);
    /*
    int16_t takeup = torque_float_to_int16(takeup_torque);
    int16_t supply = torque_float_to_int16(supply_torque);
    // Cast to uint16_t before shifting: right-shift of negative signed
    // values is implementation-defined in C.
    uint16_t takeup_u = (uint16_t)takeup;
    uint16_t supply_u = (uint16_t)supply;

    uint8_t data[5];
    // Data = [cmd_reel_torque, takeup_msb, takeup_lsb, supply_msb, supply_lsb]
    data[0] = MOTOR_COMMS_CMD_REEL_TORQUE;
    data[1] = (uint8_t)(takeup_u >> 8);
    data[2] = (uint8_t)(takeup_u & 0xFF);
    data[3] = (uint8_t)(supply_u >> 8);
    data[4] = (uint8_t)(supply_u & 0xFF);

    motor_comms_broadcast_bytes(data, 5);
    */
}

void motors_set_torque(uint8_t motor_addr, float torque) {
    int16_t torque_int = torque_float_to_int16(torque);
    // Cast to uint16_t before shifting: right-shift of negative signed
    // values is implementation-defined in C.
    uint16_t torque_u = (uint16_t)torque_int;

    // Data = [cmd_reel_torque, takeup_msb, takeup_lsb, supply_msb, supply_lsb]
    uint8_t data[3];
    data[0] = MOTOR_COMMS_CMD_REEL_TORQUE;
    data[1] = (uint8_t)(torque_u >> 8);
    data[2] = (uint8_t)(torque_u & 0xFF);

    uint8_t tx_len = 3;
    uint8_t rx_len = 0;
    uint8_t rx_buf_size = 8;
    uint8_t rx_data[8];

    RXError err = motor_comms_write_read(motor_addr, data, tx_len, rx_data, &rx_len, rx_buf_size);

    // The reel-torque reply is [cmd][meta_fault]; harvest it into the fault state.
    faults_note_motor_reply(motor_addr, err, rx_data, rx_len);
}

RXError motors_takeup_enable() {
    return confirm_command(MOTOR_COMMS_ADDR_TAKEUP, MOTOR_COMMS_CMD_ENABLE);
}

RXError motors_takeup_disable() {
    return confirm_command(MOTOR_COMMS_ADDR_TAKEUP, MOTOR_COMMS_CMD_DISABLE);
}

RXError motors_takeup_calibrate_encoder() {
    return confirm_command(MOTOR_COMMS_ADDR_TAKEUP, MOTOR_COMMS_CMD_CALIB_ENCODER);
}

RXError motors_takeup_get_faults(uint8_t* faults) {
    uint8_t buf[5];
    uint8_t buf_size = 5;
    uint8_t data_len = 0;
    RXError err = motor_comms_read(MOTOR_COMMS_ADDR_TAKEUP, MOTOR_COMMS_CMD_FAULT_STATUS, buf, &data_len, buf_size);
    if (err == RX_ERR_OK) {
        if (data_len != 2) {
            err = RX_ERR_WRONG_RESPONSE_LENGTH;
        } else {
            if (buf[0] != MOTOR_COMMS_CMD_FAULT_STATUS) {
                err = RX_ERR_WRONG_RESPONSE;
            } else {
                *faults = buf[1];
            }
        }
    }

    if (err != RX_ERR_OK) {
        *faults = 0;
    }
    return err;
}

RXError motors_supply_enable() {
    return confirm_command(MOTOR_COMMS_ADDR_SUPPLY, MOTOR_COMMS_CMD_ENABLE);
}

RXError motors_supply_disable() {
    return confirm_command(MOTOR_COMMS_ADDR_SUPPLY, MOTOR_COMMS_CMD_DISABLE);
}

RXError motors_supply_calibrate_encoder() {
    return confirm_command(MOTOR_COMMS_ADDR_SUPPLY, MOTOR_COMMS_CMD_CALIB_ENCODER);
}

RXError motors_supply_get_faults(uint8_t* faults) {
    uint8_t buf[5];
    uint8_t buf_size = 5;
    uint8_t data_len = 0;
    RXError err = motor_comms_read(MOTOR_COMMS_ADDR_SUPPLY, MOTOR_COMMS_CMD_FAULT_STATUS, buf, &data_len, buf_size);
    if (err == RX_ERR_OK) {
        if (data_len != 2) {
            err = RX_ERR_WRONG_RESPONSE_LENGTH;
        } else {
            if (buf[0] != MOTOR_COMMS_CMD_FAULT_STATUS) {
                err = RX_ERR_WRONG_RESPONSE;
            } else {
                *faults = buf[1];
            }
        }
    }

    if (err != RX_ERR_OK) {
        *faults = 0;
    }
    return err;
}

RXError motors_capstan_enable() {
    return confirm_command(MOTOR_COMMS_ADDR_CAPSTAN, MOTOR_COMMS_CMD_ENABLE);
}

RXError motors_capstan_disable() {
    return confirm_command(MOTOR_COMMS_ADDR_CAPSTAN, MOTOR_COMMS_CMD_DISABLE);
}

RXError motors_capstan_calibrate_encoder() {
    return confirm_command(MOTOR_COMMS_ADDR_CAPSTAN, MOTOR_COMMS_CMD_CALIB_ENCODER);
}

RXError motors_capstan_set_speed(CapstanSpeed speed) {
    uint8_t cmd = 0;
    switch (speed) {
        case CAPSTAN_SPEED_15IPS:
            cmd = MOTOR_COMMS_CMD_CAPSTAN_15IPS;
            break;
        case CAPSTAN_SPEED_30IPS:
            cmd = MOTOR_COMMS_CMD_CAPSTAN_30IPS;
            break;
        case CAPSTAN_SPEED_7P5_IPS:
            cmd = MOTOR_COMMS_CMD_CAPSTAN_7P5IPS;
            break;
        case CAPSTAN_SPEED_OTHER:
            return RX_ERR_TIMEOUT;
    }

    RXError err = confirm_command(MOTOR_COMMS_ADDR_CAPSTAN, cmd);
    // Capstan speed acks are a bare echo (no meta-fault byte), so this only
    // tracks the capstan's comms presence, not its meta-fault. Getting the
    // capstan's meta-fault needs an explicit FAULT_STATUS poll.
    faults_note_motor_reply(MOTOR_COMMS_ADDR_CAPSTAN, err, NULL, 0);
    return err;
}


