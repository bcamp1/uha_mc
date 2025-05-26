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
    float tension2;
    float tension1_dot;
    float tension2_dot;
} State;

typedef void (*ControllerFunc) (State, float*, float*);

typedef struct {
    ControllerFunc controller;
} ControllerConfig;

typedef struct {
    float motor1_k[11];
    float motor2_k[11];
} LinearControlLaw;

void controller_linear_control_law(LinearControlLaw* K, State* s, float* torque1, float* torque2);
State controller_get_error(State* r, State* x);

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
