#include "bldc.h"
#include "../board.h"
#include "../periphs/gpio.h"
#include "delay.h"
#include "trq_pwm.h"
#include "../periphs/spi_async.h"
#include "../periphs/spi.h"

const BLDCConfig BLDC_CONF_A = {
	.cs_pin     = PIN_BLDC_A_CS,
	.trqmag_pin = PIN_BLDC_A_TRQMAG,
	.trqdir_pin = PIN_BLDC_A_TRQDIR,
	.enable_pin = PIN_BLDC_A_ENABLE,
    .pwm_index  = PWM_INDEX_A,
};

const BLDCConfig BLDC_CONF_B = {
	.cs_pin     = PIN_BLDC_B_CS,
	.trqmag_pin = PIN_BLDC_B_TRQMAG,
	.trqdir_pin = PIN_BLDC_B_TRQDIR,
	.enable_pin = PIN_BLDC_B_ENABLE,
    .pwm_index  = PWM_INDEX_B,
};

const BLDCConfig BLDC_CONF_C = {
	.cs_pin     = PIN_BLDC_C_CS,
	.trqmag_pin = PIN_BLDC_C_TRQMAG,
	.trqdir_pin = PIN_BLDC_C_TRQDIR,
	.enable_pin = PIN_BLDC_C_ENABLE,
    .pwm_index  = PWM_INDEX_C,
};

void bldc_init_all() {
    //trq_pwm_init();
    bldc_init(&BLDC_CONF_A);
    bldc_init(&BLDC_CONF_B);
    bldc_init(&BLDC_CONF_C);
}

void bldc_init(const BLDCConfig* config) {
    gpio_init_pin(config->cs_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE); 
    //gpio_init_pin(config->trqdir_pin, GPIO_DIR_IN, GPIO_ALTERNATE_NONE); 
    gpio_init_pin(config->enable_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE); 

    gpio_clear_pin(config->cs_pin);
    gpio_clear_pin(config->trqdir_pin);
    gpio_clear_pin(config->enable_pin);
    spi_init(&SPI_CONF_BLDC_A);
}

void bldc_set_torque(const BLDCConfig* config, uint16_t torque) {
    spi_write_read16(&SPI_CONF_BLDC_A, torque);
}

/*
void bldc_set_torque(const BLDCConfig* config, float torque) {
    if (torque < 0) {
        gpio_set_pin(config->trqdir_pin);
        torque = -torque;
    } else {
        gpio_clear_pin(config->trqdir_pin);
    }

    trq_pwm_set_mag(config->pwm_index, torque);
}
*/

void bldc_enable(const BLDCConfig* config) {
    gpio_set_pin(config->enable_pin);
}

void bldc_disable(const BLDCConfig* config) {
    gpio_clear_pin(config->enable_pin);
}

void bldc_enable_all() {
    bldc_enable(&BLDC_CONF_A);
    bldc_enable(&BLDC_CONF_B);
    bldc_enable(&BLDC_CONF_C);
}

void bldc_disable_all() {
    bldc_disable(&BLDC_CONF_A);
    bldc_disable(&BLDC_CONF_B);
    bldc_disable(&BLDC_CONF_C);
}

