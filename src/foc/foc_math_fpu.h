/*
 * foc_math_fpu.h
 *
 * Created: 3/16/2025 12:46:52 AM
 *  Author: brans
 */ 


#ifndef FOC_MATH_FPU_H_
#define FOC_MATH_FPU_H_

#define PI (3.14159265f)

void park_transform(float theta, float alpha, float beta, float* d, float* q);
void inv_park_transform(float theta, float d, float q, float* alpha, float* beta);
void clark_transform(float a, float b, float c, float* alpha, float* beta);
void inv_clark_transform(float alpha, float beta, float* a, float* b, float* c);

void clark_park(float theta, float a, float b, float c, float* d, float* q);
void inv_park_clark(float theta, float d, float q, float* a, float* b, float* c);

float to_rads(float angle);
float to_degrees(float angle);
void print_float(float f);

#endif /* FOC_MATH_FPU_H_ */