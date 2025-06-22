/*
 * inc_encoder.h
 *
 * Created: 3/20/2025 10:17:50 PM
 *  Author: brans
 */ 


#ifndef INC_ENCODER_H_
#define INC_ENCODER_H_
#include <stdint.h>

#define INC_ENCODER_PULSE_PIN PIN_PB10
#define INC_ENCODER_EXT_NUM (10)
#define INC_ENCODER_DIR_PIN PIN_PB11

void inc_encoder_init();
void inc_encoder_update();
float inc_encoder_get_pos();
float inc_encoder_get_vel();

#endif /* INC_ENCODER_H_ */
