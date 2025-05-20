/*
 * inc_encoder.c
 *
 * Created: 3/20/2025 10:18:04 PM
 *  Author: brans
 */ 

#include "inc_encoder.h"
#include "../periphs/eic.h"
// #include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../periphs/timer.h"
// #include "../periphs/timer.h"
#include "stopwatch.h"
// #include "sam.h"
// #include "uha_motor_driver.h"
// #include "delay.h"

#define INC_ENCODER_TIMER_ID (0)
#define INC_ENCODER_TIMER_RATE (50.0f)
const int ppr = 1024;
static float pulse_delta_rad = 0.0f;
static float encoder_pos = 0.0f;
static float prev_encoder_pos = 0.0f;


static float prev2_encoder_vel = 0.0f;
static float prev_encoder_vel = 0.0f;
static float encoder_vel = 0.0f;
static uint32_t dt_ticks = 0;

void vel_timer_callback() {
    gpio_toggle_pin(PIN_PA14);
    float dt = stopwatch_stop(0, false);
    if (dt > 0.08f) {
        encoder_vel = 0.0f;
    }
}

float inc_encoder_get_pos() {
	return encoder_pos;
}

uint32_t inc_encoder_get_dt_ticks() {
	return dt_ticks;
}

float inc_encoder_get_vel() {
    return encoder_vel;
	//return 0.25f*prev2_encoder_vel + 0.25f*prev_encoder_vel + 0.5f*encoder_vel;
}

void inc_encoder_pulse() {
    // Get Time Elapsed
    uint32_t ticks = stopwatch_stop(0, true);
    float dt = ticks_to_time(ticks);
    dt_ticks = ticks;
    
    // Get Encoder Pos
    prev_encoder_pos = encoder_pos;
	bool dir = gpio_get_pin(INC_ENCODER_DIR_PIN);
	if (dir) {
		encoder_pos += pulse_delta_rad;
	} else {
		encoder_pos -= pulse_delta_rad;
	}

    // Calculate Velocity
    float vel = 1000.0f;
    if (dt != 0) {
        vel = (encoder_pos - prev_encoder_pos) / dt;
    }
    prev2_encoder_vel = prev_encoder_vel;
    prev_encoder_vel = encoder_vel;
    encoder_vel = vel;
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

    // Start the stopwatch
    stopwatch_start(0);
}
