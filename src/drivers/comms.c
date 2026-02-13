#include "comms.h"
#include "rs422.h"
#include "delay.h"

#define SOF_BYTE 0xAA
#define BYTE_DELAY 0xFF
#define FRAME_DELAY 0xFFF

static void comms_put(uint8_t ch) {
    rs422_put(ch);
    delay(BYTE_DELAY);
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
    delay(FRAME_DELAY);
}

