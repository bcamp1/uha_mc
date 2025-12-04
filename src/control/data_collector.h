#pragma once

typedef struct {
    float filtered_takeup_tension;
    float filtered_supply_tension;
    float takeup_tension;
    float supply_tension;
    float prev_tape_position;
    float tape_position;
    float tape_speed;
    float takeup_reel_speed;
    float supply_reel_speed;
} ControlData;

void data_collector_init();
void data_collector_update();
float data_collector_get_tape_speed();
float data_collector_get_tape_position();
float data_collector_get_takeup_tension();
float data_collector_get_supply_tension();
float data_collector_get_takeup_reel_speed();
float data_collector_get_supply_reel_speed();
void data_collector_set_reel_speeds(float takeup, float supply);

