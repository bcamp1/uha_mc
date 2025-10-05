#include "stepper.h"
#include "../board.h"
#include "../periphs/gpio.h"
#include "delay.h"

const StepperConfig STEPPER_CONF_1 = {
	.dir_pin    = PIN_STP1_DIR,
	.step_pin   = PIN_STP1_STEP,
	.enable_pin = PIN_STP1_ENABLE,
};

const StepperConfig STEPPER_CONF_2 = {
	.dir_pin    = PIN_STP2_STEP,
	.step_pin   = PIN_STP2_DIR,
	.enable_pin = PIN_STP2_ENABLE,
};

void stepper_init(const StepperConfig* config) {
    gpio_init_pin(config->dir_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_init_pin(config->step_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_init_pin(config->enable_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);

    gpio_clear_pin(config->dir_pin);
    gpio_clear_pin(config->step_pin);
    gpio_clear_pin(config->enable_pin);
}

void stepper_enable(const StepperConfig* config) {
    gpio_clear_pin(config->dir_pin);
    gpio_clear_pin(config->step_pin);
    gpio_set_pin(config->enable_pin);
}

void stepper_disable(const StepperConfig* config) {
    gpio_clear_pin(config->enable_pin);
    gpio_clear_pin(config->dir_pin);
    gpio_clear_pin(config->step_pin);
}

void stepper_send_pulses(const StepperConfig* config, int32_t steps, uint32_t delay_cycles) {
    const uint32_t on_time = 0xFF;
    gpio_clear_pin(config->step_pin);

    if (steps < 0) {
        gpio_set_pin(config->dir_pin);
        steps = -steps;
    } else {
        gpio_clear_pin(config->dir_pin);
    }

    for (int i = 0; i < steps; i++) {
        gpio_set_pin(config->step_pin);
        delay(on_time);
        gpio_clear_pin(config->step_pin);
        delay(delay_cycles);
    }
}

