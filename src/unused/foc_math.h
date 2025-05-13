/*
 * foc.h
 *
 * Created: 7/11/2024 6:49:27 PM
 *  Author: brans
 */ 


/*
#ifndef FOC_MATH_H_
#define FOC_MATH_H_

#include "fptc.h"

//#define PI fl2fpt(3.14159265)

void park_transform(fpt theta, fpt alpha, fpt beta, fpt* d, fpt* q);
void inv_park_transform(fpt theta, fpt d, fpt q, fpt* alpha, fpt* beta);
void clark_transform(fpt a, fpt b, fpt c, fpt* alpha, fpt* beta);
void inv_clark_transform(fpt alpha, fpt beta, fpt* a, fpt* b, fpt* c);

void clark_park(fpt theta, fpt a, fpt b, fpt c, fpt* d, fpt* q);
void inv_park_clark(fpt theta, fpt d, fpt q, fpt* a, fpt* b, fpt* c);

fpt to_rads(fpt angle);
fpt to_degrees(fpt angle);
void print_float(fpt f);



#endif / * FOC_MATH_H_ * /*/