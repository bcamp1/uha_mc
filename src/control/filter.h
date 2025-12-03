#ifndef FILTER_H
#define FILTER_H

typedef struct {
    // Numerator coefficients (b(z))
    float b0;
    float b1;
    float b2;

    // Denominator coefficients (a(z))
    float a0;
    float a1;
    float a2;

    // State variables
    float x_k;
    float x_kminus1;
    float x_kminus2;
    float y_k;
    float y_kminus1;
    float y_kminus2;
} Filter;

// Initialize filter with coefficients
// H(z) = (b0 + b1*z^-1 + b2*z^-2) / (a0 + a1*z^-1 + a2*z^-2)
void filter_init(Filter* filter, float b0, float b1, float b2, float a0, float a1, float a2);

// Reset filter state to zero
void filter_reset(Filter* filter);

// Compute next filter output
float filter_next(float x_k, Filter* filter);

// Initialize filter as a PD controller (position form)
// Kp: Proportional gain
// Kd: Derivative gain
// T: Sample time (seconds)
void filter_init_pd(Filter* filter, float Kp, float Kd, float T);

// Initialize filter as a zero filter (always outputs 0)
void filter_init_zero(Filter* filter);

// Initialize filter from s-plane transfer function using bilinear transformation
// H(s) = (num2*s^2 + num1*s + num0) / (den2*s^2 + den1*s + den0)
// num2, num1, num0: numerator coefficients (s^2, s^1, s^0)
// den2, den1, den0: denominator coefficients (s^2, s^1, s^0)
// T: sample time (seconds)
void filter_init_s_plane(Filter* filter, float num2, float num1, float num0,
                         float den2, float den1, float den0, float T);

#endif
