#include "simple_filter.h"

float simple_filter_next(float u, SimpleFilter* filter) {
    float y_k = ((1.0f - filter->alpha)*u) + filter->alpha * filter->y_kminus1;
    filter->y_kminus1 = y_k;
    return y_k;
}

void simple_filter_clear(SimpleFilter* filter) {
    filter->y_kminus1 = 0.0f;
}

SimpleFilter simple_filter_create(float alpha) {
    return (SimpleFilter) {
        .alpha = alpha,
        .y_kminus1 = 0.0f,
    };
}

