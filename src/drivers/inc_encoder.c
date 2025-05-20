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
#define INC_ENCODER_TIMER_RATE (50.0f)
#define INC_ENCODER_PPR (1024)

const float rad_tick_amount = 0.006135923151542565f;
const float dt_ticks_to_time = 2.13248e-06f;

static int32_t pulses = 0;
static uint16_t dt_ticks = 0;

static void init_tcc();
static void init_evsys();
static uint16_t tcc_get_elapsed();

float inc_encoder_get_pos() {
	return rad_tick_amount * (float) pulses;
}

float inc_encoder_get_vel() {
    float dt = dt_ticks_to_time * (float) dt_ticks;
    if (dt < 0.00001) dt = 0.00001;
    float vel = (2*rad_tick_amount) / dt;
    return (float) dt_ticks;
}

uint16_t inc_encoder_get_dt_ticks() {
	return dt_ticks;
}

void inc_encoder_pulse() {
    // Get Time Elapsed
    dt_ticks = tcc_get_elapsed();
    
    // Get Encoder Pos
	bool dir = gpio_get_pin(INC_ENCODER_DIR_PIN);
	if (dir) {
        pulses++;
	} else {
        pulses--;
	}
}

void inc_encoder_init() {
    // Init event manager
    init_evsys();

	// Init EIC
	eic_init();

    // Init Control Timer 
    init_tcc();
	
	// Init pulse eic pin
	eic_init_pin(INC_ENCODER_PULSE_PIN, INC_ENCODER_EXT_NUM, EIC_MODE_BOTH, inc_encoder_pulse);	
	
	// Init dir pin
	gpio_init_pin(INC_ENCODER_DIR_PIN, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
	
	// Init velocity timer
	// timer_schedule(INC_ENCODER_TIMER_ID, INC_ENCODER_TIMER_RATE, vel_timer_callback);

    // Start the stopwatch
    //stopwatch_start(0);
}

#define TIMER TCC2
#define TCC_PRESCALER (256)
static void init_tcc() {
    // Enable Clocks
	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2;
	//GCLK->PCHCTRL[GCLK_TCC2_TCC3_INDEX].reg |= GCLK_PCHCTRL_CHEN;
    GCLK->PCHCTRL[TCC2_GCLK_ID].reg = GCLK_PCHCTRL_GEN_GCLK0 | GCLK_PCHCTRL_CHEN;
    // Disable timer
    TIMER->CTRLA.bit.ENABLE = 0;
    while (TIMER->SYNCBUSY.bit.ENABLE) {}

	// Set prescaler
	//TIMER->CTRLA.reg |= TCC_PRESCALER << 8;	// Set prescaler
	TIMER->CTRLA.reg |= TCC_CTRLA_PRESCALER_DIV64;	// Set prescaler

    // Set capture on channel 0
    TIMER->CTRLA.reg |= TCC_CTRLA_CPTEN0 | TCC_CTRLA_CPTEN1;

    // Configure to use EV[0] as capture and retrigger source
    TIMER->EVCTRL.reg = TCC_EVCTRL_MCEI0 | TCC_EVCTRL_MCEI1 | TCC_EVCTRL_TCEI1 | TCC_EVCTRL_EVACT1_PPW; 

    TCC2->PER.reg = 0xFFFFFF;
    while (TCC2->SYNCBUSY.bit.PER);

    TIMER->WAVE.reg = TCC_WAVE_WAVEGEN_NFRQ;  // Normal frequency (for up-counting)
    while (TCC2->SYNCBUSY.bit.WAVE);

    TIMER->CTRLA.bit.ENABLE = 1;
    while (TIMER->SYNCBUSY.bit.ENABLE) {}
    
    // Start timer
    //TIMER->CTRLBSET.bit.CMD |= TCC_CTRLBSET_CMD_RETRIGGER;
    //while (TIMER->SYNCBUSY.bit.CTRLB);   // Wait for sync
}

static uint16_t tcc_get_elapsed() {
    //delay(0xF);
    //TIMER->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC; // Request sync of COUNT
    while(TIMER->SYNCBUSY.bit.CC0) {}
    while(TIMER->SYNCBUSY.bit.CC1) {}
    uint16_t T = TIMER->CC[0].reg;
    //uint16_t pulse_width = TIMER->CC[1].reg;
    //uart_print("T: ");
    //uart_print_int_base(T, 16);
    //uart_print(", pulse_width: ");
    //uart_println_int_base(pulse_width, 16);
    return T;
}

static void init_evsys() {
    // Enable EVSYS
    MCLK->APBBMASK.bit.EVSYS_ = 1;

    // Connect EIC Channel 0 to EVSYS Channel 0
    EVSYS->Channel[0].CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
        EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
        EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_10);

    // Connect EVSYS Channel 0 to TCC0 event input
    EVSYS->USER[EVSYS_ID_USER_TCC2_EV_1].reg = EVSYS_USER_CHANNEL(0 + 1); // Channel 0 is USER 1
}

