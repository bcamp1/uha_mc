#include "roller.h"
#include "inc_encoder.h"

//#define ROLLER_RADIUS_INCHES (0.79f)
#define ROLLER_RADIUS_INCHES (1.0f)
#define PI (3.14159f)

void roller_init() {
    inc_encoder_init();
}

float roller_get_ips() {
    return ROLLER_RADIUS_INCHES * inc_encoder_get_vel(); 
}

// Returns tape position in seconds given an ips standard
float roller_get_tape_position(float ips) {
    float total_tape_inches = ROLLER_RADIUS_INCHES * inc_encoder_get_pos();
    return total_tape_inches / ips;
}

