#include "user_comms.h"
#include "rs422.h"
#include <stdbool.h>
#include <string.h>

#define SOF_BYTE 0xAA

static volatile bool comms_rx_pending = false;

static void on_rx_ready(void) {
    comms_rx_pending = true;
}

void comms_init() {
    rs422_init(on_rx_ready);
}

void comms_send_bytes(const uint8_t *data, uint8_t length) {
    uint8_t checksum = length;  // seed with length
    for (uint16_t i = 0; i < length; i++)
        checksum += data[i];

    uint8_t frame[3 + 255];
    frame[0] = SOF_BYTE;
    frame[1] = length;
    memcpy(&frame[2], data, length);
    frame[2 + length] = checksum;

    rs422_send_bytes(frame, 3 + length);
}


void comms_send_float(const uint8_t command, float data) {
    uint8_t buf[5];
    buf[0] = command;
    memcpy(&buf[1], &data, 4);
    comms_send_bytes(buf, 5);
}

static bool get_byte_with_timeout(uint8_t* byte, uint32_t timeout) {
    for (uint32_t i = 0; i < timeout; i++) {
        int ch = rs422_get();
        if (ch != -1) {
            *byte = (uint8_t)ch;
            return true;
        }
    }
    return false;
}

bool comms_get_data(uint8_t* data, uint8_t* data_len, uint8_t buf_size) {
    int ch;

    do {
        ch = rs422_get();
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

float comms_data_to_float(uint8_t* data) {
    float result;
    memcpy(&result, data, 4);
    return result;
}
