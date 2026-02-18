#include "comms.h"
#include "rs422.h"
#include "delay.h"

#define SOF_BYTE 0xAA
#define BYTE_DELAY 0xFF
#define FRAME_DELAY 0xFFF

static void comms_put(uint8_t ch) {
    rs422_put(ch);
}

void comms_init() {
    rs422_init();
}

void comms_send_bytes(const uint8_t *data, uint8_t length) {
    uint8_t checksum = length;  // seed with length
    for (uint16_t i = 0; i < length; i++)
        checksum += data[i];

    comms_put(SOF_BYTE);
    comms_put(length);
    for (uint16_t i = 0; i < length; i++) {
        comms_put(data[i]);
    }
    comms_put(checksum);
}


static bool get_byte_with_timeout(uint8_t* byte, uint32_t timeout) {
    for (uint32_t i = 0; i < timeout; i++) {
        int16_t ch = rs422_get();
        if (ch != -1) {
            *byte = (uint8_t)ch;
            return true;
        }
    }
    return false;
}

bool comms_get_data(uint8_t* data, uint8_t* data_len, uint8_t buf_size) {
    int16_t ch;

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

