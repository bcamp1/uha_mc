#pragma once

#include <stdint.h>

// rx_ready (nullable) is invoked from the RXC ISR every time a byte enters the
// RX ring. Callers can pass a stub that sets a flag or feeds a state machine.
void rs422_init(void (*rx_ready)(void));

// RX (async ring, ISR-safe)
int  rs422_get(void);          // returns 0..255, or -1 if empty
int  rs422_available(void);
void rs422_rx_flush(void);

// TX (async ring). Returns once queued; DRE/TXC ISRs handle shift-out.
// Callers MUST run at lower priority than PRIO_RS422 or risk deadlock when the
// ring is full.
void rs422_send_bytes(const uint8_t* data, uint16_t len);
