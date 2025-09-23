#pragma once

#include <stdint.h>
#include "../periphs/spi.h"

#define BLDC_IDENT_UNKNOWN  (0b00)
#define BLDC_IDENT_SUPPLY   (0b10)
#define BLDC_IDENT_TAKEUP   (0b01)
#define BLDC_IDENT_CAPSTAN  (0b11)

typedef struct {
    uint8_t ident_code;
	uint8_t cs_pin;
	uint8_t ident1_pin;
	uint8_t ident0_pin;
	uint8_t enable_pin;
    const SPIConfig* spi_conf; 
} BLDCConfig;

extern const BLDCConfig BLDC_CONF_A;
extern const BLDCConfig BLDC_CONF_B;

void bldc_init(const BLDCConfig* config);
void bldc_enable(const BLDCConfig* config);
void bldc_disable(const BLDCConfig* config);
void bldc_set_torque(const BLDCConfig* config, uint16_t torque);
void bldc_init_all();
void bldc_enable_all();
void bldc_disable_all();

