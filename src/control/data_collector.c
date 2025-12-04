#include "data_collector.h"
#include "../sched.h"
#include "filter.h"
#include "simple_filter.h"
#include "../drivers/tension_arm.h"
#include "../drivers/inc_encoder.h"

static volatile ControlData data;

// Filters
static SimpleFilter tape_speed_filter;
static SimpleFilter takeup_tension_filter;
static SimpleFilter supply_tension_filter;

// Filter Parameters
static const float tape_speed_filter_alpha = 0.99f;
static const float tension_arm_filter_alpha = 0.9f;

// Sample Period
static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);

void data_collector_init() {
    // Initialize Data
    data.filtered_takeup_tension = 0.0f;
    data.filtered_supply_tension = 0.0f;
    data.takeup_tension = 0.0f;
    data.supply_tension = 0.0f;
    data.prev_tape_position = 0.0f;
    data.tape_position = 0.0f;
    data.tape_speed = 0.0f;
    data.takeup_reel_speed = 0.0f;
    data.supply_reel_speed = 0.0f;

    // Initialize Filters
    tape_speed_filter = simple_filter_create(tape_speed_filter_alpha);
    takeup_tension_filter = simple_filter_create(tension_arm_filter_alpha);
    supply_tension_filter = simple_filter_create(tension_arm_filter_alpha);
}

void data_collector_update() {
    // New tensions
    float tension_t = tension_arm_get_position(&TENSION_ARM_A);
    float tension_s = tension_arm_get_position(&TENSION_ARM_B);
    float takeup_tension = simple_filter_next(tension_t, &takeup_tension_filter);
    float supply_tension = simple_filter_next(tension_s, &supply_tension_filter);
    
    // New tape speed
    data.prev_tape_position = data.tape_position;
    float tape_position = inc_encoder_get_position();
    float tape_delta = (tape_position - data.prev_tape_position) / T;
    float tape_speed = simple_filter_next(tape_delta, &tape_speed_filter);

    data.tape_position = tape_position;
    data.tape_speed = tape_speed;

    data.filtered_takeup_tension = takeup_tension;
    data.filtered_supply_tension = supply_tension;
    data.takeup_tension = tension_t;
    data.supply_tension = tension_s;
}

float data_collector_get_tape_speed() {
    return data.tape_speed;
}

float data_collector_get_tape_position() {
    return data.tape_position;
}

float data_collector_get_takeup_tension() {
    return data.filtered_takeup_tension;
}

float data_collector_get_supply_tension() {
    return data.filtered_supply_tension;
}

float data_collector_get_takeup_reel_speed() {
    return data.takeup_reel_speed;
}

float data_collector_get_supply_reel_speed() {
    return data.supply_reel_speed;
}

void data_collector_set_reel_speeds(float takeup, float supply) {
    data.takeup_reel_speed = takeup;
    data.supply_reel_speed = supply;
}

