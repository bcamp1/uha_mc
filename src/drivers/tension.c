/*
 * ems22a.c
 *
 * Created: 3/20/2025 12:35:23 AM
 *  Author: brans
 */ 

#include "tension.h"
#include "../periphs/spi.h"

// Last raw 16-bit frame read from each EMS22A, cached by the *_raw() readers so
// the fault layer can judge the arm without issuing its own SPI transaction (a
// second, async read corrupts the shared SERCOM3 bus the 200 Hz control loop
// depends on). Written from the control-loop ISR, read from the main loop; a
// 16-bit aligned access is atomic on Cortex-M. Seed with a valid frame
// (non-stuck, even parity) so the arms read healthy until the first real sample
// lands.
static volatile uint16_t last_raw_takeup = 0x00C0;
static volatile uint16_t last_raw_supply = 0x00C0;

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
    if (fraction > 1.0f) fraction = 1.0f;
    return fraction;
}

void tension_init() {
	spi_init(&SPI_CONF_TENSION_TAKEUP);
	spi_init(&SPI_CONF_TENSION_SUPPLY);

    if (takeup_bottom < takeup_top) {
        takeup_bottom += 1.0f;
    }
}

void tension_init_takeup_only() {
	spi_init(&SPI_CONF_TENSION_TAKEUP);

    if (takeup_bottom < takeup_top) {
        takeup_bottom += 1.0f;
    }
}

void tension_init_supply_only() {
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
	last_raw_takeup = result;
	uint16_t position = result >> 6;
    float max = 0x03FF;
    return ((float) position) / max;
}

float tension_get_supply() {
    return interpolate(supply_bottom, supply_top, tension_get_supply_raw());
}

float tension_get_supply_raw() {
	uint16_t result = spi_write_read16(&SPI_CONF_TENSION_SUPPLY, 0x00);
	last_raw_supply = result;
	uint16_t position = result >> 6;
    float max = 0x03FF;
    return ((float) position) / max;
}

// The EMS22A appends an even-parity bit (bit 0) over the rest of the frame, so a
// valid frame has an even number of set bits overall. XOR-fold to test it.
static bool parity_ok(uint16_t raw) {
    raw ^= raw >> 8;
    raw ^= raw >> 4;
    raw ^= raw >> 2;
    raw ^= raw >> 1;
    return (raw & 1u) == 0u;  // even parity -> all bits XOR to 0
}

// True when the cached frame indicates a fault rather than a good reading:
//   - stuck line (0x0000/0xFFFF): a disconnected/unpowered arm floats MISO
//   - bad parity: a powered arm whose frame came back corrupted
// Inspects only the cached frame: no SPI, safe from any context.
static bool raw_is_error(uint16_t raw) {
    if (raw == 0x0000 || raw == 0xFFFF) return true;
    if (!parity_ok(raw)) return true;
    return false;
}

bool tension_takeup_error() {
    return raw_is_error(last_raw_takeup);
}

bool tension_supply_error() {
    return raw_is_error(last_raw_supply);
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

