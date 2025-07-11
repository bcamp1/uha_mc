/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include "tension_arm.h"
#include "spi_collector.h"

const TensionArmConfig TENSION_ARM_A = {
	.spi = &SPI_CONF_TENSION_ARM_A,
	.top_position = 466,
	.bottom_position = 588,
};

const TensionArmConfig TENSION_ARM_B = {
	.spi = &SPI_CONF_TENSION_ARM_B,
	.top_position = 725,
	.bottom_position = 606,
};

void tension_arm_init(const TensionArmConfig* config) {
	// Init SPI
	spi_init(config->spi);
}

float tension_arm_get_position(const TensionArmConfig* config) {
	uint16_t result;
    if (config->spi == &SPI_CONF_TENSION_ARM_A) {
        result = spi_collector_get_tension_a();
    } else {
        result = spi_collector_get_tension_b();
    }
	uint16_t position = result >> 6;
	
	// Interpolate between max and min
	float interpolation = ((float) (config->bottom_position - position)) / ((float) (config->top_position - config->bottom_position));
	if (interpolation < 0) {
		interpolation = -interpolation;
	}
	return interpolation;
}

