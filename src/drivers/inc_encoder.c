/*
 * inc_encoder.c
 *
 * Created: 3/20/2025 10:18:04 PM
 *  Author: brans
 */ 

#include "inc_encoder.h"
#include "../periphs/eic.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../periphs/timer.h"
#include "sam.h"
#include "uha_motor_driver.h"

#define INC_ENCODER_TIMER_ID (0)
#define INC_ENCODER_TIMER_RATE (50.0f)

const int ppr = 1024;
static float pulse_delta_rad = 0.0f;
static float encoder_pos = 0.0f;
static float encoder_vel = 0.0f;

static void vel_timer_callback() {
	//Suart_println("HI");
	static float prev_pos = 0.0f;
	float new_pos = encoder_pos;
	
	encoder_vel = INC_ENCODER_TIMER_RATE * (new_pos - prev_pos);
	prev_pos = new_pos;
	gpio_toggle_pin(UHA_MTR_DRVR_CONF_B.en);
}

float inc_encoder_get_pos() {
	return encoder_pos;
}

float inc_encoder_get_vel() {
	return encoder_vel;
}

void inc_encoder_pulse() {
	bool dir = gpio_get_pin(INC_ENCODER_DIR_PIN);
	if (dir) {
		encoder_pos += pulse_delta_rad;
	} else {
		encoder_pos -= pulse_delta_rad;
	}
}

void inc_encoder_init() {
	// Set pulse delta
	pulse_delta_rad = 2.0f*3.14159f*(1.0f/((float) ppr));
	
	// Init EIC
	eic_init();
	
	// Init pulse eic pin
	eic_init_pin(INC_ENCODER_PULSE_PIN, INC_ENCODER_EXT_NUM, EIC_MODE_BOTH, inc_encoder_pulse);	
	
	// Init dir pin
	gpio_init_pin(INC_ENCODER_DIR_PIN, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
	
	// Init velocity timer
	timer_schedule(INC_ENCODER_TIMER_ID, INC_ENCODER_TIMER_RATE, vel_timer_callback);
}
