#ifndef CONTROLLER_H
#define CONTROLLER_H

#define CONTROLLER_TIMER_ID (1)
#define CONTROLLER_IPS_TARGET (15.0f)

typedef struct {
    float time;
    float theta1;
    float theta2;
    float theta1_dot;
    float theta2_dot;
    float tape_position;
    float tape_speed;
    float tension1;
    float tension1_dot;
    float tension2;
    float tension2_dot;
} State;

typedef void (*ControllerFunc) (State, float*, float*);

typedef struct {
    ControllerFunc controller;
    float sample_period;
} ControllerConfig;

void controller_init_all_hardware();
void controller_disable_motors();
void controller_enable_motors();
void controller_set_config(ControllerConfig* c);
void controller_run_iteration();
State controller_get_state();
void controller_send_state_uart();
void controller_print_tension_info();
void controller_print_encoder_info();
void controller_start_process();
void controller_stop_process();
float controller_get_time();

#endif
