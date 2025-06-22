#include "simple_filter.h"

float simple_filter_next(float u, SimpleFilter* filter) {
    float y_k = ((1.0f - filter->alpha)*u) + filter->alpha * filter->y_kminus1;
    filter->y_kminus1 = y_k;
    return y_k;
}

