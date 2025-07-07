#pragma once
#include <stdint.h>

void spi_collector_init();
void spi_collector_enable_service();
void spi_collector_disable_service();

float spi_collector_get_encoder_a();
float spi_collector_get_encoder_b();
float spi_collector_get_encoder_a_pole();
float spi_collector_get_encoder_b_pole();
float spi_collector_get_tension_a();
float spi_collector_get_tension_b();

void spi_collector_set_torque_a(float torque);
void spi_collector_set_torque_b(float torque);
void spi_collector_enable_motors();
void spi_collector_disable_motors();

