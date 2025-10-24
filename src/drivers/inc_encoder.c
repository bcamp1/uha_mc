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
#include "../board.h"
#include <sam.h>

#define GCLK_TCC0_TCC1_INDEX (25)
#define GCLK_TCC2_TCC3_INDEX (29)

#define INC_ENCODER_TIMER_ID (0)
#define INC_ENCODER_TIMER_RATE (5.0f)
#define INC_ENCODER_PPR (1024)

#define STOPWATCH_PRECISION (5)

static volatile const float rad_tick_amount = 0.006135923151542565f;

static volatile int32_t pulses = 0;

int32_t inc_encoder_get_ticks() {
    return pulses;
}

float inc_encoder_get_pos() {
	return rad_tick_amount * (float) pulses;
}

static void inc_encoder_pulse() {
    gpio_set_pin(PIN_DEBUG1);
    // Get Encoder Pos
	bool dir = gpio_get_pin(PIN_ROLLER_DIR);
	if (dir) {
        pulses++;
	} else {
        pulses--;
	}
    gpio_clear_pin(PIN_DEBUG1);
}

void inc_encoder_init() {
    eic_init();
	
	// Init pulse eic pin
	eic_init_pin(PIN_ROLLER_PULSE, 2, EIC_MODE_BOTH, inc_encoder_pulse);	
	
	// Init dir pin
	gpio_init_pin(PIN_ROLLER_DIR, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);

    gpio_set_pin(PIN_DEBUG1);
}


