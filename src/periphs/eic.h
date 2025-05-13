/*
 * eic.h
 *
 * Created: 3/20/2025 9:46:12 PM
 *  Author: brans
 */ 

#include "sam.h"

#ifndef EIC_H_
#define EIC_H_

typedef void (*func_ptr_t)();

void eic_init();
void eic_init_pin(uint16_t pin, uint16_t ext_num, uint16_t int_mode, func_ptr_t callback);

// Interrupt modes
#define EIC_MODE_NONE	(0)
#define EIC_MODE_RISE	(1)
#define EIC_MODE_FALL	(2)
#define EIC_MODE_BOTH	(3)
#define EIC_MODE_HIGH	(4)
#define EIC_MODE_LOW	(5)

#endif /* EIC_H_ */