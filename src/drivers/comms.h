#pragma once
#include <stdint.h>
#include <stdbool.h>

void comms_init();
void comms_send_bytes(const uint8_t *data, uint8_t length);


