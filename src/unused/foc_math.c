/*
 * foc.c
 *
 * Created: 7/11/2024 6:49:18 PM
 *  Author: brans
 */ 

/*
#include "foc_math.h"

fpt to_rads(fpt angle) {
	return fpt_mul(angle, fl2fpt(0.0174533));
}

fpt to_degrees(fpt angle) {
	return fpt_mul(angle, fl2fpt(57.2958));
}

void print_float(fpt f) {
	//printf("%s\n", fpt_cstr(f, 5));
}

void park_transform(fpt theta, fpt alpha, fpt beta, fpt* d, fpt* q) {
	fpt sin_theta = fpt_sin(theta);
	fpt cos_theta = fpt_cos(theta);

	*d = fpt_add(fpt_mul(alpha, cos_theta), fpt_mul(beta, sin_theta));
	*q = fpt_sub(fpt_mul(beta, cos_theta), fpt_mul(alpha, sin_theta));
}

void inv_park_transform(fpt theta, fpt d, fpt q, fpt* alpha, fpt* beta) {
	fpt sin_theta = fpt_sin(theta);
	fpt cos_theta = fpt_cos(theta);

	*alpha = fpt_sub(fpt_mul(d, cos_theta), fpt_mul(q, sin_theta));
	*beta = fpt_add(fpt_mul(q, cos_theta), fpt_mul(d, sin_theta));
}

void clark_transform(fpt a, fpt b, fpt c, fpt* alpha, fpt* beta) {
	fpt b_minus_c = fpt_sub(b, c);
	fpt b_plus_c = fpt_add(b, c);
	*alpha = fpt_sub(fpt_mul(FPT_TWO_THIRDS, a), fpt_mul(FPT_ONE_THIRD, b_plus_c));
	*beta = fpt_mul(b_minus_c, FPT_INV_RT_THREE);
}

void inv_clark_transform(fpt alpha, fpt beta, fpt* a, fpt* b, fpt* c) {
	fpt sqrt_three_beta = fpt_mul(fl2fpt(1.73205080757), beta);
	fpt neg_alpha = fpt_mul(FPT_MINUS_ONE, alpha);
	
	*a = alpha;
	*b = fpt_div(fpt_add(neg_alpha, sqrt_three_beta), FPT_TWO);
	*c = fpt_div(fpt_sub(neg_alpha, sqrt_three_beta), FPT_TWO);
}

void clark_park(fpt theta, fpt a, fpt b, fpt c, fpt* d, fpt* q) {
	fpt alpha, beta;
	clark_transform(a, b, c, &alpha, &beta);
	park_transform(theta, alpha, beta, d, q);
}

void inv_park_clark(fpt theta, fpt d, fpt q, fpt* a, fpt* b, fpt* c) {
	fpt alpha, beta;
	inv_park_transform(theta, d, q, &alpha, &beta);
	inv_clark_transform(alpha, beta, a, b, c);
}

*/
