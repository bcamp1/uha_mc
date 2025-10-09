#include "stepper.h"
#include "../board.h"
#include "../periphs/gpio.h"
#include "delay.h"

const StepperConfig STEPPER_CONF_CAPSTAN = {
	.dir_pin    = PIN_STP1_DIR,
	.step_pin   = PIN_STP1_STEP,
	.enable_pin = PIN_STP1_ENABLE,
};

const StepperConfig STEPPER_CONF_TAPE_LIFTER = {
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

static bool capstan_engaged = false;
static bool tape_lifter_engaged = false;

void stepper_capstan_init() {
    capstan_engaged = false;
    stepper_init(&STEPPER_CONF_CAPSTAN);
    delay(0xFFFF);
    stepper_enable(&STEPPER_CONF_CAPSTAN);
    stepper_send_pulses(&STEPPER_CONF_CAPSTAN, 200, 0x8F);
    stepper_send_pulses(&STEPPER_CONF_CAPSTAN, 400, 0x8FF);
    stepper_send_pulses(&STEPPER_CONF_CAPSTAN, -180, 0x8F);
}

void stepper_capstan_engage() {
    if (!capstan_engaged) {
        stepper_send_pulses(&STEPPER_CONF_CAPSTAN, 182, 0x8F);
        capstan_engaged = true;
    }
}

void stepper_capstan_disengage() {
    if (capstan_engaged) {
        stepper_send_pulses(&STEPPER_CONF_CAPSTAN, -182, 0x8F);
        capstan_engaged = false;
    }

}

void stepper_tape_lifter_init() {

}

void stepper_tape_lifter_engage() {

}

void stepper_tape_lifter_disengage() {

}

