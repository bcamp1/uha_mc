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
#include "component/tcc.h"
 #include "delay.h"
#include "instance/evsys.h"
#include "stopwatch.h"
#include <sam.h>

#define GCLK_TCC0_TCC1_INDEX (25)
#define GCLK_TCC2_TCC3_INDEX (29)

#define INC_ENCODER_TIMER_ID (0)
#define INC_ENCODER_TIMER_RATE (10.0f)
#define INC_ENCODER_PPR (1024)

#define STOPWATCH_PRECISION (30)

static const float rad_tick_amount = 0.006135923151542565f;

static int32_t pulses = 0;
static uint16_t stopwatch_ticks = 0;
static int32_t stopwatch_start_pos = 0;

static float vel = 0.0f;

float inc_encoder_get_pos() {
	return rad_tick_amount * (float) pulses;
}

float inc_encoder_get_vel() {
    return vel;
}

static void inc_encoder_pulse() {
    // Get Encoder Pos
	bool dir = gpio_get_pin(INC_ENCODER_DIR_PIN);
	if (dir) {
        pulses++;
	} else {
        pulses--;
	}

    // Stopwatch stuff
    stopwatch_ticks++;
    if (stopwatch_ticks >= STOPWATCH_PRECISION) {
        stopwatch_ticks = 0;
        float dt = ticks_to_time(stopwatch_stop(0, true));
        int32_t delta = pulses - stopwatch_start_pos;
        vel = ((float) delta * rad_tick_amount) / dt;
        stopwatch_start_pos = pulses;
    }


}

static void vel_timer_callback() {
    if (ticks_to_time(stopwatch_stop(0, false)) > 0.1f) {
        vel = 0.0f;
    }
}

void inc_encoder_init() {
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


