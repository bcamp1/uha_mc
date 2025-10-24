/*
 * inc_encoder.h
 *
 * Created: 3/20/2025 10:17:50 PM
 *  Author: brans
 */ 


#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_
#include <stdint.h>
#include "../board.h"

#define INC_ENCODER_EXT_NUM (2)
#define INC_ENCODER_PULSE_PIN PIN_ROLLER_PULSE
#define INC_ENCODER_DIR_PIN PIN_ROLLER_DIR

void inc_encoder_init();
float inc_encoder_get_pos();
int32_t inc_encoder_get_ticks();

#endif /* INC_ENCODER_H_ */
