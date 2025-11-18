#pragma once

#include <stdint.h>

typedef struct {
	uint8_t pin;
} SolenoidConfig;

extern const SolenoidConfig SOLENOID_CONF_PINCH;
extern const SolenoidConfig SOLENOID_CONF_LIFTER;

void solenoid_init(const SolenoidConfig* config);
void solenoid_engage(const SolenoidConfig* config);
void solenoid_disengage(const SolenoidConfig* config);

void solenoid_pinch_init();
void solenoid_pinch_engage();
void solenoid_pinch_disengage();

void solenoid_lifter_init();
void solenoid_lifter_engage();
void solenoid_lifter_disengage();

