#include "motor_comms.h"
#include "rs485.h"
#include <string.h>

#define SOF_BYTE 0xAA

void motor_comms_init() {
    rs485_init();
}

// Frame layout: [SOF][addr][length][checksum][data...]
// Checksum is seeded with addr + length and covers the data bytes.
void motor_comms_send_bytes(uint8_t addr, const uint8_t *data, uint8_t length) {
    uint8_t checksum = addr + length;  // seed with addr + length
    for (uint16_t i = 0; i < length; i++)
        checksum += data[i];

    uint8_t frame[4 + 255];
    frame[0] = SOF_BYTE;
    frame[1] = addr;
    frame[2] = length;
    frame[3] = checksum;
    memcpy(&frame[4], data, length);

    rs485_send_bytes(frame, 4 + length);
}


void motor_comms_send_cmd(uint8_t addr, uint8_t cmd) {
    motor_comms_send_bytes(addr, &cmd, 1);
}

void motor_comms_send_float(uint8_t addr, const uint8_t command, float data) {
    uint8_t buf[5];
    buf[0] = command;
    memcpy(&buf[1], &data, 4);
    motor_comms_send_bytes(addr, buf, 5);
}

static bool get_byte_with_timeout(uint8_t* byte, uint32_t timeout) {
    for (uint32_t i = 0; i < timeout; i++) {
        int16_t ch = rs485_get();
        if (ch != -1) {
            *byte = (uint8_t)ch;
            return true;
        }
    }
    return false;
}

bool motor_comms_get_data(uint8_t* data, uint8_t* data_len, uint8_t buf_size) {
    int16_t ch;

    do {
        ch = rs485_get();
    } while (ch != -1 && ch != SOF_BYTE);

    if (ch == -1) {
        return false;
    }

    // get length
    uint8_t length = 0;
    if (!get_byte_with_timeout(&length, 0xFFF)) {
       return false;
    }
    if (length > buf_size) return false;

    // get data
    uint8_t data_byte = 0;
    uint8_t checksum = length;
    for (uint32_t i = 0; i < length; i++) {
        if (!get_byte_with_timeout(&data_byte, 0xFFF)) {
           return false;
        }

        data[i] = data_byte;
        checksum += data_byte;
    }

    // verify checksum
    uint8_t given_checksum = 0;
    if (!get_byte_with_timeout(&given_checksum, 0xFFF)) {
       return false;
    }

    if (given_checksum != checksum) return false;

    *data_len = length;
    return true;
}

float motor_comms_data_to_float(uint8_t* data) {
    float result;
    memcpy(&result, data, 4);
    return result;
}
