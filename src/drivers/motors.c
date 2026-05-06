#include "motors.h"
#include "motor_comms.h"
#include <stdint.h>

// +1.0f -> INT16_MAX, -1.0f -> INT16_MIN. Asymmetric scaling because the
// negative range has one more representable value than the positive range.
static int16_t torque_float_to_int16(float torque) {
    if (torque >= 1.0f)  return INT16_MAX;
    if (torque <= -1.0f) return INT16_MIN;
    if (torque >= 0.0f)  return (int16_t)(torque * 32767.0f);
    return (int16_t)(torque * 32768.0f);
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
}

void motors_takeup_enable() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_TAKEUP, MOTOR_COMMS_CMD_ENABLE);
}

void motors_takeup_disable() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_TAKEUP, MOTOR_COMMS_CMD_DISABLE);
}

void motors_takeup_calibrate_encoder() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_TAKEUP, MOTOR_COMMS_CMD_CALIB_ENCODER);
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

void motors_supply_enable() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_SUPPLY, MOTOR_COMMS_CMD_ENABLE);
}

void motors_supply_disable() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_SUPPLY, MOTOR_COMMS_CMD_DISABLE);
}

void motors_supply_calibrate_encoder() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_SUPPLY, MOTOR_COMMS_CMD_CALIB_ENCODER);
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

void motors_capstan_enable() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_CAPSTAN, MOTOR_COMMS_CMD_ENABLE);
}

void motors_capstan_disable() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_CAPSTAN, MOTOR_COMMS_CMD_DISABLE);
}

void motors_capstan_calibrate_encoder() {
    motor_comms_send_cmd(MOTOR_COMMS_ADDR_CAPSTAN, MOTOR_COMMS_CMD_CALIB_ENCODER);
}

void motors_capstan_set_speed(CapstanSpeed speed) {
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
            return;
    }

    motor_comms_send_cmd(MOTOR_COMMS_ADDR_CAPSTAN, cmd);
}


