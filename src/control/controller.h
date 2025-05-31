#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "sensor_state.h"

#define CONTROLLER_TIMER_ID (1)

typedef void (*ControllerFunc) (SensorState, SensorState, SensorState, SensorState, float*, float*);

typedef struct {
    float theta1_dot;
    float theta2_dot;
    float tape_speed;
    float tension1;
    float tension2;
} ControllerReference;

typedef struct {
    ControllerFunc controller;
    ControllerReference* reference;
} ControllerConfig;

void controller_init_all_hardware();
void controller_disable_motors();
void controller_enable_motors();
void controller_set_config(ControllerConfig* c);
void controller_run_iteration();
void controller_send_state_uart();
void controller_start_process();
void controller_stop_process();
void controller_print_state();
float controller_get_time();

#endif
