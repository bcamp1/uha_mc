/*
 * pwm.c
 *
 * Created: 7/10/2024 6:24:51 PM
 *  Author: brans
 */ 

#include "trq_pwm.h"
#include "../periphs/gpio.h"
#include "sam.h"

#define DSCRITICAL		(0x4)
#define DSBOTTOM		(0x5)
#define DSBOTH			(0x6)
#define DSTOP			(0x7)
#define TCC_CMD_START	(0x1)
#define TCC_CMD_STOP	(0x2)

#define PWM_PRESCALER	(0x4)

// Timer Settings
#define TIMER_TOP		(0xFF)

// Clock
#define GCLK_TCC0_TCC1_INDEX (25)
#define GCLK_TCC2_TCC3_INDEX (29)

/*
Initialization steps:
1. Enable clock
2. CPTEN
3. CTRLA.PRESCALER
4. CTRLA.PRESCSYNC
5. WAVE.WAVEGEN
6. WAVE.POL
7. DRVCTRL.INVEN
*/
void trq_pwm_init() {
	// Init Pins
	//init_pwm_pins();
	gpio_init_pin(PWM_PIN_A, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PWM_PIN_B, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	gpio_init_pin(PWM_PIN_C, GPIO_DIR_OUT, GPIO_ALTERNATE_G_TCC_PDEC);
	
	// Configure clock
	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0 | MCLK_APBBMASK_TCC1;
	
	// Enable GCLK0 to Peripheral
	GCLK->PCHCTRL[GCLK_TCC0_TCC1_INDEX].reg |= GCLK_PCHCTRL_CHEN;
 	
	// Configure timer
	PWM_TIMER->CTRLA.reg |= PWM_PRESCALER << 8;	// Set prescaler
	
	// Set Wavegen
    PWM_TIMER->WAVE.reg |= DSTOP 
		| TCC_WAVE_POL0
		| TCC_WAVE_POL1
		| TCC_WAVE_POL2
		| TCC_WAVE_POL3;

	PWM_TIMER->PER.reg = TIMER_TOP;
	
	PWM_TIMER->CTRLA.bit.ENABLE = 1; // Enable timer
	
	// timer->CTRLBSET.reg = TCC_CTRLBSET_CMD_RETRIGGER | TCC_CTRLBSET_CMD_READSYNC | TCC_CTRLBSET_DIR;
	PWM_TIMER->CTRLBSET.reg = TCC_CTRLBSET_CMD_READSYNC;
		
	while (PWM_TIMER->SYNCBUSY.bit.CTRLB); // Wait for busy
}

void trq_pwm_set_all_mags(float a, float b, float c) {
    trq_pwm_set_mag(PWM_INDEX_A, a);
    trq_pwm_set_mag(PWM_INDEX_B, b);
    trq_pwm_set_mag(PWM_INDEX_C, c);
}

void trq_pwm_set_mag(uint16_t index, float mag) {
    uint8_t mag_int = (uint16_t) (mag * 255.0f);
    PWM_TIMER->CC[index].reg = mag_int;
}

