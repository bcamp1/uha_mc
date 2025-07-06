#pragma once
#include <stdint.h>

void spi_collector_init();
void spi_collector_start_service();

float spi_collector_get_encoder_a();
float spi_collector_get_encoder_b();
float spi_collector_get_encoder_a_pole();
float spi_collector_get_encoder_b_pole();
float spi_collector_get_tension_a();
float spi_collector_get_tension_b();

