/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include "tension.h"
#include "../periphs/spi.h"

static float takeup_bottom = 0.003f;
static float takeup_top = 0.8856f;

static float supply_bottom = 0.1476f;
static float supply_top = 0.2639f;

// Performs a - b
static float angle_distance(float a, float b) {
    float diff = a - b;
    if (diff > 0.5f) diff -= 1.0f;
    else if (diff < -0.5f) diff += 1.0f;
    return diff;
}

static float interpolate(float bottom, float top, float value) {
    float fraction = angle_distance(value, bottom) / angle_distance(top, bottom);
    if (fraction < 0) fraction = -fraction;
    return fraction;
}

void tension_init() {
	spi_init(&SPI_CONF_TENSION_TAKEUP);
	spi_init(&SPI_CONF_TENSION_SUPPLY);

    if (takeup_bottom < takeup_top) {
        takeup_bottom += 1.0f;
    }
}

// Information
float tension_get_takeup() {
    return interpolate(takeup_bottom, takeup_top, tension_get_takeup_raw());
}

float tension_get_takeup_raw() {
	uint16_t result = spi_write_read16(&SPI_CONF_TENSION_TAKEUP, 0x00);
	uint16_t position = result >> 6;
    float max = 0x03FF;
    return ((float) position) / max;
}

float tension_get_supply() {
    return interpolate(supply_bottom, supply_top, tension_get_supply_raw());
}

float tension_get_supply_raw() {
	uint16_t result = spi_write_read16(&SPI_CONF_TENSION_SUPPLY, 0x00);
	uint16_t position = result >> 6;
    float max = 0x03FF;
    return ((float) position) / max;
}


// Calibration
void tension_calibrate_takeup_bottom() {

}

void tension_calibrate_takeup_top() {

}

void tension_calibrate_supply_bottom() {

}

void tension_calibrate_supply_top() {

}

