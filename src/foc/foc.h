/*
 * foc.h
 *
 * Created: 7/30/2024 5:36:46 PM
 *  Author: brans
 */ 


#ifndef FOC_H_
#define FOC_H_

#include <stdint.h>

void foc_get_duties(float theta, float d, float q, float* a, float* b, float* c);

#endif /* FOC_H_ */