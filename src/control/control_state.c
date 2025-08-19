#include "control_state.h"
#include "../periphs/uart.h"
#include "../drivers/motor_encoder.h"
#include "../drivers/inc_encoder.h"
#include "../drivers/spi_collector.h"
#include "../drivers/roller.h"
#include <stdbool.h>

#define TWOPI (6.28318f)
#define PI (3.14159f)

static float sub_angles(float x, float y) {
    float a = x - y;
    float b = (x + TWOPI) - y;
    float c = x - (y + TWOPI);

    float abs_a = a;
    float abs_b = b;
    float abs_c = c;

    if (abs_a < 0.0f) abs_a = -abs_a;
    if (abs_b < 0.0f) abs_b = -abs_b;
    if (abs_c < 0.0f) abs_c = -abs_c;

    if (abs_a <= abs_b && abs_a <= abs_c) {
        return a;
    } else if (abs_b <= abs_a && abs_b <= abs_c) {
        return b;
    } else {
        return c;
    }
}

static volatile float prev_takeup_reel_theta = 0.0f;
static volatile float prev_takeup_reel_speed = 0.0f;
static volatile float prev_takeup_tension = 0.0f;

static volatile float prev_supply_reel_theta = 0.0f;
static volatile float prev_supply_reel_speed = 0.0f;
static volatile float prev_supply_tension = 0.0f;

static volatile float prev_tape_speed = 0.0f;

ControlState control_state_get_filtered_state(const ControlStateFilter* filter, float sample_rate) {
    float takeup_reel_theta = spi_collector_get_encoder_a();
    float supply_reel_theta = spi_collector_get_encoder_b();
    float takeup_tension = spi_collector_get_tension_a();
    float supply_tension = spi_collector_get_tension_b();

    // Calculate reel speeds
    float takeup_reel_speed = sample_rate * sub_angles(takeup_reel_theta, prev_takeup_reel_theta); 
    float supply_reel_speed = sample_rate * sub_angles(supply_reel_theta, prev_supply_reel_theta); 

    // Calculate tension arm speeds
    float takeup_tension_speed = sample_rate * (takeup_tension - prev_takeup_tension);
    float supply_tension_speed = sample_rate * (supply_tension - prev_supply_tension);

    // Get roller encoder data
    inc_encoder_update();
    float tape_position = roller_get_tape_position(15.0f);
    float tape_speed = roller_get_ips();

    // Apply smoothing filters to speeds
    takeup_reel_speed = simple_filter_next(takeup_reel_speed, filter->takeup_reel_speed); 
    supply_reel_speed = simple_filter_next(supply_reel_speed, filter->supply_reel_speed); 
    takeup_tension_speed = simple_filter_next(takeup_tension_speed, filter->takeup_tension_speed); 
    supply_tension_speed = simple_filter_next(supply_tension_speed, filter->supply_tension_speed); 
    tape_speed = simple_filter_next(tape_speed, filter->tape_speed);

    // Compute accelerations
    float takeup_reel_acceleration = sample_rate * (takeup_reel_speed - prev_takeup_reel_speed);
    float supply_reel_acceleration = sample_rate * (supply_reel_speed - prev_supply_reel_speed);
    float tape_acceleration = sample_rate * (tape_speed - prev_tape_speed);

    // Filter accelerations
    takeup_reel_acceleration = simple_filter_next(takeup_reel_acceleration, filter->takeup_reel_acceleration);
    supply_reel_acceleration = simple_filter_next(supply_reel_acceleration, filter->supply_reel_acceleration);
    tape_acceleration = simple_filter_next(tape_acceleration, filter->tape_acceleration);

    // Set previous variables
    prev_takeup_reel_theta = takeup_reel_theta;
    prev_takeup_reel_speed = takeup_reel_speed;
    prev_takeup_tension = takeup_tension;

    prev_supply_reel_theta = supply_reel_theta;
    prev_supply_reel_speed = supply_reel_speed;
    prev_supply_tension = supply_tension;

    prev_tape_speed = tape_speed;

    // Create struct
    ControlState state = {
         .takeup_reel_speed        = takeup_reel_speed,
         .takeup_reel_acceleration = takeup_reel_acceleration,
                                   
         .takeup_tension           = takeup_tension,
         .takeup_tension_speed     = takeup_tension_speed,
                                   
         .supply_reel_speed        = supply_reel_speed,
         .supply_reel_acceleration = supply_reel_acceleration,
                                   
         .supply_tension           = supply_tension,
         .supply_tension_speed     = supply_tension_speed,
                                   
         .tape_position            = tape_position,
         .tape_speed               = tape_speed,
         .tape_acceleration        = tape_acceleration,
    };

    return state;
}

void control_state_transmit_uart(float time, ControlState state) {
    bool send_takeup_reel_speed         = true;
    bool send_takeup_reel_acceleration  = false;
    bool send_takeup_tension            = true;
    bool send_takeup_tension_speed      = false;
    bool send_supply_reel_speed         = true;
    bool send_supply_reel_acceleration  = false;
    bool send_supply_tension            = true;
    bool send_supply_tension_speed      = false;
    bool send_tape_position             = false;
    bool send_tape_speed                = true;
    bool send_tape_acceleration         = false;
    int data_count = 0;
    float data[20] = {0};
    data[data_count] = time;
    data_count++;
    if (send_takeup_reel_speed) {data[data_count] = state.takeup_reel_speed; data_count++;}
    if (send_takeup_reel_acceleration) {data[data_count] = state.takeup_reel_acceleration; data_count++;}
    if (send_takeup_tension) {data[data_count] = state.takeup_tension; data_count++;}
    if (send_takeup_tension_speed) {data[data_count] = state.takeup_tension_speed; data_count++;}
    if (send_supply_reel_speed) {data[data_count] = state.supply_reel_speed; data_count++;}
    if (send_supply_reel_acceleration) {data[data_count] = state.supply_reel_acceleration; data_count++;}
    if (send_supply_tension) {data[data_count] = state.supply_tension; data_count++;}
    if (send_supply_tension_speed) {data[data_count] = state.supply_tension_speed; data_count++;}
    if (send_tape_position) {data[data_count] = state.tape_position; data_count++;}
    if (send_tape_speed) {data[data_count] = state.tape_speed; data_count++;}
    if (send_tape_acceleration) {data[data_count] = state.tape_acceleration; data_count++;}
    uart_println_float_arr(data, data_count);
}

ControlState control_state_sub(ControlState* a, ControlState* b) {
    return (ControlState) {
        .takeup_reel_speed = a->takeup_reel_speed - b->takeup_reel_speed,
        .takeup_reel_acceleration = a->takeup_reel_acceleration - b->takeup_reel_acceleration,

        .takeup_tension = a->takeup_tension - b->takeup_tension,
        .takeup_tension_speed = a->takeup_tension_speed - b->takeup_tension_speed,

        .supply_reel_speed = a->supply_reel_speed - b->supply_reel_speed,
        .supply_reel_acceleration = a->supply_reel_acceleration - b->supply_reel_acceleration,

        .supply_tension = a->supply_tension - b->supply_tension,
        .supply_tension_speed = a->supply_tension_speed - b->supply_tension_speed,

        .tape_position = a->tape_position - b->tape_position,
        .tape_speed = a->tape_speed - b->tape_speed,
        .tape_acceleration = a->tape_acceleration - b->tape_acceleration,
    };
}

float control_state_dot(ControlState* a, ControlState* b) {
    return (a->takeup_reel_speed * b->takeup_reel_speed)
        + (a->takeup_reel_acceleration * b->takeup_reel_acceleration)
        + (a->takeup_tension * b->takeup_tension)
        + (a->takeup_tension_speed * b->takeup_tension_speed)
        + (a->supply_reel_speed * b->supply_reel_speed)
        + (a->supply_reel_acceleration * b->supply_reel_acceleration)
        + (a->supply_tension * b->supply_tension)
        + (a->supply_tension_speed * b->supply_tension_speed)
        + (a->tape_position * b->tape_position)
        + (a->tape_speed * b->tape_speed)
        + (a->tape_acceleration * b->tape_acceleration);
}

