#pragma once
#include <stdint.h>

void spi_collector_init();
void spi_collector_start_service();

uint16_t spi_collector_get_encoder_a();
uint16_t spi_collector_get_encoder_b();
uint16_t spi_collector_get_tension_a();
uint16_t spi_collector_get_tension_b();

