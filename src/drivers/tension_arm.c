/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include "tension_arm.h"
#include "../periphs/spi.h"

const TensionArmConfig TENSION_ARM_A = {
	.spi = &SPI_CONF_TENSION_A,
	.top_position = 372,
	.bottom_position = 494,
};

const TensionArmConfig TENSION_ARM_B = {
	.spi = &SPI_CONF_TENSION_B,
	.top_position = 778,
	.bottom_position = 657,
};

void tension_arm_init(const TensionArmConfig* config) {
	// Init SPI
	spi_init(config->spi);
}

float tension_arm_get_position(const TensionArmConfig* config) {
	uint16_t result = spi_write_read16(config->spi, 0x00);
	uint16_t position = result >> 6;

    //return (float) position;
	
	// Interpolate between max and min
	float interpolation = ((float) (config->bottom_position - position)) / ((float) (config->top_position - config->bottom_position));
	if (interpolation < 0) {
		interpolation = -interpolation;
	}
	return interpolation;
}

