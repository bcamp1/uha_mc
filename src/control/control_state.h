#ifndef CONTROL_STATE_H
#define CONTROL_STATE_H

#include "simple_filter.h"

typedef struct {
    float takeup_reel_speed;
    float takeup_reel_acceleration;

    float takeup_tension;
    float takeup_tension_speed;

    float supply_reel_speed;
    float supply_reel_acceleration;

    float supply_tension;
    float supply_tension_speed;

    float tape_position;
    float tape_speed;
    float tape_acceleration;
} ControlState;

typedef struct {
    SimpleFilter* takeup_reel_speed;
    SimpleFilter* takeup_reel_acceleration;
    SimpleFilter* takeup_tension_speed;

    SimpleFilter* supply_reel_speed;
    SimpleFilter* supply_reel_acceleration;
    SimpleFilter* supply_tension_speed;

    SimpleFilter* tape_speed;
    SimpleFilter* tape_acceleration;
} ControlStateFilter;

ControlState control_state_get_filtered_state(const ControlStateFilter* filter, float sample_rate);
void control_state_transmit_uart(float time, ControlState state);
ControlState control_state_sub(ControlState* a, ControlState* b);
float control_state_dot(ControlState* a, ControlState* b);

#endif

