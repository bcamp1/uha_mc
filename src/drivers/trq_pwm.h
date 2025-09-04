/*
 * pwm.h
 *
 * Created: 7/10/2024 6:25:10 PM
 *  Author: brans
 */ 


#ifndef PWM_H_
#define PWM_H_

#include <stdint.h>
#include "sam.h"
#include "../board.h"

#define PWM_INDEX_A (3)
#define PWM_INDEX_B (2)
#define PWM_INDEX_C (1)

#define PWM_PIN_A PIN_BLDC_A_TRQMAG
#define PWM_PIN_B PIN_BLDC_B_TRQMAG
#define PWM_PIN_C PIN_BLDC_C_TRQMAG
#define PWM_TIMER TCC0

void trq_pwm_init();
void trq_pwm_set_all_mags(float a, float b, float c);
void trq_pwm_set_mag(uint16_t index, float mag);

#endif /* PWM_H_ */
