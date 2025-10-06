#include "bldc.h"
#include "../board.h"
#include "../periphs/gpio.h"
#include "delay.h"
#include "../periphs/spi_async.h"
#include "../periphs/spi.h"

#define SPI_SPEED_MAX (40.0f)

const BLDCConfig BLDC_CONF_CAPSTAN = {
    .ident_code = BLDC_IDENT_CAPSTAN,
	.cs_pin     = PIN_BLDC_C_CS,
	.ident1_pin = PIN_BLDC_C_IDENT1,
	.ident0_pin = PIN_BLDC_C_IDENT0,
	.enable_pin = PIN_BLDC_C_ENABLE,
    .spi_conf = &SPI_CONF_BLDC_C,
};

const BLDCConfig BLDC_CONF_TAKEUP = {
    .ident_code = BLDC_IDENT_TAKEUP,
	.cs_pin     = PIN_BLDC_A_CS,
	.ident1_pin = PIN_BLDC_A_IDENT1,
	.ident0_pin = PIN_BLDC_A_IDENT0,
	.enable_pin = PIN_BLDC_A_ENABLE,
    .spi_conf = &SPI_CONF_BLDC_A,
};

const BLDCConfig BLDC_CONF_SUPPLY = {
    .ident_code = BLDC_IDENT_SUPPLY,
	.cs_pin     = PIN_BLDC_B_CS,
	.ident1_pin = PIN_BLDC_B_IDENT1,
	.ident0_pin = PIN_BLDC_B_IDENT0,
	.enable_pin = PIN_BLDC_B_ENABLE,
    .spi_conf = &SPI_CONF_BLDC_B,
};

void bldc_init_all() {
    bldc_init(&BLDC_CONF_CAPSTAN);
    bldc_init(&BLDC_CONF_TAKEUP);
    bldc_init(&BLDC_CONF_SUPPLY);
}

void bldc_init(const BLDCConfig* config) {
    gpio_init_pin(config->cs_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE); 
    gpio_init_pin(config->ident1_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE); 
    gpio_init_pin(config->ident0_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE); 
    gpio_init_pin(config->enable_pin, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE); 

    gpio_clear_pin(config->cs_pin);
    gpio_clear_pin(config->enable_pin);
    
    uint8_t ident1 = config->ident_code >> 1;
    uint8_t ident0 = config->ident_code & 0b01;

    if (ident1) {
        gpio_set_pin(config->ident1_pin);
    } else {
        gpio_clear_pin(config->ident1_pin);
    }

    if (ident0) {
        gpio_set_pin(config->ident0_pin);
    } else {
        gpio_clear_pin(config->ident0_pin);
    }

    spi_init(config->spi_conf);
    bldc_set_torque_float(config, 0.0f);
}

/*
int16_t bldc_set_torque(const BLDCConfig* config, int16_t torque) {
    return (int16_t) spi_write_read16(config->spi_conf, (uint16_t) torque);
}
*/

float bldc_set_torque_float(const BLDCConfig* config, float torque) {
    if (torque > 1.0f) torque = 1.0f;
    if (torque < -1.0f) torque = -1.0f;
    int16_t torque_int = (int16_t) (torque * 32760.0f);
    int16_t speed_int = spi_write_read16(config->spi_conf, (uint16_t) torque_int);
    float speed = (((float)speed_int) / 32767.0f) * SPI_SPEED_MAX;
    return speed;
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
    bldc_enable(&BLDC_CONF_CAPSTAN);
    bldc_enable(&BLDC_CONF_TAKEUP);
    bldc_enable(&BLDC_CONF_SUPPLY);
}

void bldc_disable_all() {
    bldc_disable(&BLDC_CONF_CAPSTAN);
    bldc_disable(&BLDC_CONF_TAKEUP);
    bldc_disable(&BLDC_CONF_SUPPLY);
}

