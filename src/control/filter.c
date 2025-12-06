#include "filter.h"

void filter_init(Filter* filter, float b0, float b1, float b2, float a0, float a1, float a2) {
    // Set coefficient values
    filter->b0 = b0;
    filter->b1 = b1;
    filter->b2 = b2;
    filter->a0 = a0;
    filter->a1 = a1;
    filter->a2 = a2;

    // Initialize state variables to zero
    filter->x_k = 0.0f;
    filter->x_kminus1 = 0.0f;
    filter->x_kminus2 = 0.0f;
    filter->y_k = 0.0f;
    filter->y_kminus1 = 0.0f;
    filter->y_kminus2 = 0.0f;
}

void filter_reset(Filter* filter) {
    filter->x_k = 0.0f;
    filter->x_kminus1 = 0.0f;
    filter->x_kminus2 = 0.0f;
    filter->y_k = 0.0f;
    filter->y_kminus1 = 0.0f;
    filter->y_kminus2 = 0.0f;
}

float filter_next(float x_k, Filter* filter) {
    // Shift input state history: k-2 <- k-1 <- k <- x_k
    filter->x_kminus2 = filter->x_kminus1;
    filter->x_kminus1 = filter->x_k;
    filter->x_k = x_k;

    // Compute output using difference equation:
    // y[n] = (b0*x[n] + b1*x[n-1] + b2*x[n-2] - a1*y[n-1] - a2*y[n-2]) / a0
    float y_k = (filter->b0 * filter->x_k +
                 filter->b1 * filter->x_kminus1 +
                 filter->b2 * filter->x_kminus2 -
                 filter->a1 * filter->y_kminus1 -
                 filter->a2 * filter->y_kminus2) / filter->a0;

    // Shift output state history: k-2 <- k-1 <- k <- y_k
    filter->y_kminus2 = filter->y_kminus1;
    filter->y_kminus1 = filter->y_k;
    filter->y_k = y_k;

    return y_k;
}

void filter_init_pd(Filter* filter, float Kp, float Kd, float T) {
    // Position form PD controller (no integral action)
    // u[k] = Kp*e[k] + Kd*(e[k] - e[k-1])/T
    // H(z) = Kp + (Kd/T)*(1 - z^-1)
    //      = (Kp + Kd/T) - (Kd/T)*z^-1

    float b0 = Kp + Kd / T;
    float b1 = -Kd / T;
    float b2 = 0.0f;

    // No integrator in denominator
    float a0 = 1.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;

    filter_init(filter, b0, b1, b2, a0, a1, a2);
}

Filter filter_from_pd(float Kp, float Kd, float T) {
    Filter f;
    filter_init_pd(&f, Kp, Kd, T);
    return f;
}

void filter_init_zero(Filter* filter) {
    // Zero filter: H(z) = 0
    // Output is always zero regardless of input
    float b0 = 0.0f;
    float b1 = 0.0f;
    float b2 = 0.0f;
    float a0 = 1.0f;
    float a1 = 0.0f;
    float a2 = 0.0f;

    filter_init(filter, b0, b1, b2, a0, a1, a2);
}

void filter_init_s_plane(Filter* filter, float num2, float num1, float num0,
                         float den2, float den1, float den0, float T) {
    // Bilinear (Tustin) transformation: s = (2/T) * (1 - z^-1) / (1 + z^-1)
    //
    // Given H(s) = (num2*s^2 + num1*s + num0) / (den2*s^2 + den1*s + den0)
    // Convert to H(z) = (b0 + b1*z^-1 + b2*z^-2) / (a0 + a1*z^-1 + a2*z^-2)

    float w = 2.0f / T;  // Bilinear transformation frequency warping factor
    float w2 = w * w;    // w squared

    // Compute unnormalized numerator coefficients
    // After substitution and simplification: multiply out (1+z^-1)^2
    float B0 = num2 * w2 + num1 * w + num0;
    float B1 = 2.0f * num0 - 2.0f * num2 * w2;
    float B2 = num2 * w2 - num1 * w + num0;

    // Compute unnormalized denominator coefficients
    float A0 = den2 * w2 + den1 * w + den0;
    float A1 = 2.0f * den0 - 2.0f * den2 * w2;
    float A2 = den2 * w2 - den1 * w + den0;

    // Normalize by A0 to get standard form with a0 = 1
    float b0 = B0 / A0;
    float b1 = B1 / A0;
    float b2 = B2 / A0;
    float a0 = 1.0f;
    float a1 = A1 / A0;
    float a2 = A2 / A0;

    filter_init(filter, b0, b1, b2, a0, a1, a2);
}
