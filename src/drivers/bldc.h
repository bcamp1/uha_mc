#pragma once

#include <stdint.h>

typedef struct {
	uint8_t cs_pin;
	uint8_t trqmag_pin;
	uint8_t trqdir_pin;
	uint8_t enable_pin;
    uint8_t pwm_index;
} BLDCConfig;

extern const BLDCConfig BLDC_CONF_A;
extern const BLDCConfig BLDC_CONF_B;

void bldc_init(const BLDCConfig* config);
void bldc_enable(const BLDCConfig* config);
void bldc_disable(const BLDCConfig* config);
void bldc_set_torque(const BLDCConfig* config, float torque);
void bldc_init_all();
void bldc_enable_all();
void bldc_disable_all();
void bldc_set_all_torques(float torque_a, float torque_b, float torque_c);

