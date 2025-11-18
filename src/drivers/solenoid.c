#include "solenoid.h"
#include "../periphs/gpio.h"
#include "../board.h"

const SolenoidConfig SOLENOID_CONF_PINCH = {
	.pin = PIN_PINCH_SOLENOID,
};

const SolenoidConfig SOLENOID_CONF_LIFTER = {
	.pin = PIN_LIFT_SOLENOID,
};

void solenoid_init(const SolenoidConfig* config) {
    gpio_init_pin(config->pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
}

void solenoid_engage(const SolenoidConfig* config) {
    gpio_set_pin(config->pin);
}

void solenoid_disengage(const SolenoidConfig* config) {
    gpio_clear_pin(config->pin);
}

void solenoid_pinch_init() {
    solenoid_init(&SOLENOID_CONF_PINCH);
}

void solenoid_pinch_engage() {
    solenoid_engage(&SOLENOID_CONF_PINCH);
}

void solenoid_pinch_disengage() {
    solenoid_disengage(&SOLENOID_CONF_PINCH);
}

void solenoid_lifter_init() {
    solenoid_init(&SOLENOID_CONF_LIFTER);
}

void solenoid_lifter_engage() {
    solenoid_engage(&SOLENOID_CONF_LIFTER);
}

void solenoid_lifter_disengage() {
    solenoid_disengage(&SOLENOID_CONF_LIFTER);
}

