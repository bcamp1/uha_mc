/*
 * timer.h
 *
 * Created: 3/21/2025 2:20:49 AM
 *  Author: brans
 */ 


#ifndef TIMER_H_
#define TIMER_H_

#include <stdint.h>

typedef void (*func_ptr_t)();

void timer_schedule(uint16_t timer_id, float sample_rate, func_ptr_t callback);
void timer_deschedule(uint16_t timer_id);
void timer_init_all();

#endif /* TIMER_H_ */
