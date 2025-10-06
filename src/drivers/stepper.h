#pragma once

#include <stdint.h>

typedef struct {
	uint8_t dir_pin;
	uint8_t step_pin;
	uint8_t enable_pin;
} StepperConfig;

extern const StepperConfig STEPPER_CONF_CAPSTAN;
extern const StepperConfig STEPPER_CONF_TAPE_LIFTER;

void stepper_init(const StepperConfig* config);
void stepper_enable(const StepperConfig* config);
void stepper_disable(const StepperConfig* config);
void stepper_send_pulses(const StepperConfig* config, int32_t steps, uint32_t delay_cycles);
void stepper_capstan_init();
void stepper_capstan_engage();
void stepper_capstan_disengage();
void stepper_tape_lifter_init();
void stepper_tape_lifter_engage();
void stepper_tape_lifter_disengage();

