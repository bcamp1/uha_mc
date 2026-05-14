#include "user_comms.h"
#include "rs422.h"
#include "samd51j20a.h"
#include <string.h>
#include <stdint.h>
#include <stdbool.h>

// Wire format: [SOF][length][checksum][data...]
// Checksum is seeded with length and covers the data bytes.

#define SOF_BYTE 0xAA

typedef enum {
    ST_SOF,
    ST_LEN,
    ST_CKSUM,
    ST_DATA,
    ST_DROP_TAIL,  // Frame is too big; drain remaining bytes to stay aligned
} RxState;

static volatile RxState  state = ST_SOF;
static volatile uint8_t  expected_len = 0;
static volatile uint8_t  given_checksum = 0;
static volatile uint8_t  calc_checksum = 0;
static volatile uint8_t  data_idx = 0;
static volatile uint8_t  scratch[COMMS_MAX_DATA];
static volatile uint16_t drop_remaining = 0;

static volatile CommsRxResult latched;
static volatile bool          result_ready = false;

static CommsMessageReadyCb message_ready_cb = 0;

static void latch_success(void) {
    latched.err = RX_ERR_OK;
    latched.data_len = expected_len;
    for (uint8_t i = 0; i < expected_len; i++) {
        latched.data[i] = scratch[i];
    }
    result_ready = true;
    if (message_ready_cb != 0) {
        message_ready_cb();
    }
}

// Surface a parse-time error to the consumer. Does not touch `state` —
// caller decides whether to resync immediately (ST_SOF) or drain a tail.
static void latch_error(RXError err) {
    latched.err = err;
    latched.data_len = 0;
    result_ready = true;
    if (message_ready_cb != 0) {
        message_ready_cb();
    }
}

static void finalize_checksum(void) {
    if (given_checksum == calc_checksum) {
        latch_success();
    } else {
        latch_error(RX_ERR_WRONG_CHECKSUM);
    }
    state = ST_SOF;
}

static void feed(uint8_t byte) {
    switch (state) {
        case ST_SOF:
            if (byte == SOF_BYTE) {
                state = ST_LEN;
            }
            break;
        case ST_LEN:
            if (byte > COMMS_MAX_DATA) {
                // Skip the cksum byte plus `byte` cargo bytes, then resync.
                drop_remaining = (uint16_t)byte + 1;
                state = ST_DROP_TAIL;
                latch_error(RX_ERR_BUF_OVF);
                return;
            }
            expected_len = byte;
            calc_checksum = byte;
            data_idx = 0;
            state = ST_CKSUM;
            break;
        case ST_CKSUM:
            given_checksum = byte;
            if (expected_len == 0) {
                finalize_checksum();
            } else {
                state = ST_DATA;
            }
            break;
        case ST_DATA:
            scratch[data_idx++] = byte;
            calc_checksum += byte;
            if (data_idx == expected_len) {
                finalize_checksum();
            }
            break;
        case ST_DROP_TAIL:
            if (--drop_remaining == 0) {
                state = ST_SOF;
            }
            break;
    }
}

static void on_rx_ready(void) {
    int ch;
    while ((ch = rs422_get()) != -1) {
        feed((uint8_t)ch);
    }
}

void comms_init(void) {
    state = ST_SOF;
    drop_remaining = 0;
    result_ready = false;
    rs422_init(on_rx_ready);
}

// Outgoing frame: [SOF][length][checksum][data...]
void comms_send_bytes(const uint8_t *data, uint8_t length) {
    if (length > COMMS_MAX_DATA) return;

    uint8_t checksum = length;
    for (uint16_t i = 0; i < length; i++) {
        checksum += data[i];
    }

    uint8_t frame[3 + COMMS_MAX_DATA];
    frame[0] = SOF_BYTE;
    frame[1] = length;
    frame[2] = checksum;
    memcpy(&frame[3], data, length);

    rs422_send_bytes(frame, 3 + length);
}

void comms_send_byte(uint8_t data) {
    comms_send_bytes(&data, 1);
}

void comms_send_float(uint8_t command, float data) {
    uint8_t buf[5];
    buf[0] = command;
    memcpy(&buf[1], &data, 4);
    comms_send_bytes(buf, 5);
}

void  comms_send_float2(uint8_t command, float data1, float data2) {
    uint8_t buf[9];
    buf[0] = command;
    memcpy(&buf[1], &data1, 4);
    memcpy(&buf[5], &data2, 4);
    comms_send_bytes(buf, 9);
}

void comms_register_message_ready_cb(CommsMessageReadyCb cb) {
    message_ready_cb = cb;
}

CommsRxResult comms_get_data(void) {
    if (!result_ready) {
        CommsRxResult empty = { .err = RX_ERR_NO_DATA, .data_len = 0 };
        return empty;
    }

    NVIC_DisableIRQ(SERCOM0_2_IRQn);
    CommsRxResult out;
    out.err = latched.err;
    out.data_len = latched.data_len;
    for (uint8_t i = 0; i < latched.data_len; i++) {
        out.data[i] = latched.data[i];
    }
    result_ready = false;
    NVIC_EnableIRQ(SERCOM0_2_IRQn);
    return out;
}

float comms_data_to_float(const uint8_t* data) {
    float result;
    memcpy(&result, data, 4);
    return result;
}
