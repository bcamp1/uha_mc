#include "sensor_state.h"
#include <math.h>

#define TWOPI (6.283185307179586f)
#define PI (3.14159f)

static float sub_angles(float x, float y) {
    float a = x - y;
    float b = (x + TWOPI) - y;
    float c = x - (y + TWOPI);

    float abs_a = fabs(a);
    float abs_b = fabs(b);
    float abs_c = fabs(c);

    if (abs_a <= abs_b && abs_a <= abs_c) {
        return a;
    } else if (abs_b <= abs_a && abs_b <= abs_c) {
        return b;
    } else {
        return c;
    }
}

static float add_angles(float x, float y) {
    float z = x + y;

    if (z < 0) {
        return z + TWOPI;
    }

    if (z > TWOPI) {
        return z - TWOPI;
    }

    return z;
}

SensorState sensor_state_add(SensorState a, SensorState b) {
    float* a_vec = a.state;
    float* b_vec = b.state;
    return (SensorState) {
        .state = {
            add_angles(a_vec[0], b_vec[0]),
            add_angles(a_vec[1], b_vec[1]),
            a_vec[2] + b_vec[2],
            a_vec[3] + b_vec[3],
            a_vec[4] + b_vec[4],
            a_vec[5] + b_vec[5],
        }
    };
}

SensorState sensor_state_sub(SensorState a, SensorState b) {
    float* a_vec = a.state;
    float* b_vec = b.state;
    return (SensorState) {
        .state = {
            sub_angles(a_vec[0], b_vec[0]),
            sub_angles(a_vec[1], b_vec[1]),
            a_vec[2] - b_vec[2],
            a_vec[3] - b_vec[3],
            a_vec[4] - b_vec[4],
            a_vec[5] - b_vec[5],
        }
    };
}

SensorState sensor_state_scale(SensorState a, float c) {
    float* a_vec = a.state;
    return (SensorState) {
        .state = {
            a_vec[0] * c,
            a_vec[1] * c,
            a_vec[2] * c,
            a_vec[3] * c,
            a_vec[4] * c,
            a_vec[5] * c,
        }
    };
}

float sensor_state_dot(SensorState a, float* k) {
    float* a_vec = a.state;
    return
        a_vec[0] * k[0] + 
        a_vec[1] * k[1] + 
        a_vec[2] * k[2] + 
        a_vec[3] * k[3] + 
        a_vec[4] * k[4] + 
        a_vec[5] * k[5]; 
}
