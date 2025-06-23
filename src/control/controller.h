#ifndef CONTROLLER_H
#define CONTROLLER_H

#include "control_state.h"

#define CONTROLLER_TIMER_ID (1)

typedef void (*ControllerFunc) (ControlState, float*, float*);

typedef struct {
    ControllerFunc controller;
    ControlState* reference;
} ControllerConfig;

void controller_init_all_hardware();
void controller_disable_motors();
void controller_enable_motors();
void controller_set_config(ControllerConfig* c);
void controller_run_iteration();
void controller_send_state_uart();
void controller_start_process();
void controller_stop_process();
float controller_get_time();

#endif

