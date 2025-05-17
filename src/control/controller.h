#ifndef CONTROLLER_H
#define CONTROLLER_H

typedef struct {
    float theta1;
    float theta2;
    float theta1_dot;
    float theta2_dot;
    float tape_speed;
    float tension1;
    float tension1_dot;
    float tension2;
    float tension2_dot;
} State;

typedef float (*ControllerFunc) (State);

typedef struct {
    ControllerFunc controller1;
    ControllerFunc controller2;
    float sample_period;
} ControllerConfig;

void controller_init_all_hardware();
void controller_set_config(ControllerConfig* c);
void controller_run_iteration();
void controller_run_loop();
State controller_get_state();
void controller_send_state_uart();

#endif