#pragma once

#include <stdint.h>

typedef struct {
	uint8_t dir_pin;
	uint8_t step_pin;
	uint8_t enable_pin;
} StepperConfig;

extern const StepperConfig STEPPER_CONF_1;
extern const StepperConfig STEPPER_CONF_2;

void stepper_init(const StepperConfig* config);
void stepper_enable(const StepperConfig* config);
void stepper_disable(const StepperConfig* config);
void stepper_send_pulses(const StepperConfig* config, int32_t steps, uint32_t delay_cycles);

