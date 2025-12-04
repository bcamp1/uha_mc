#include "movement.h"

static float integrator = 0.0f;
static MovementDirection direction = FORWARD;
static float time = 0.0f;

static float apply_torque(float torque);
static float get_tape_speed();

static float apply_torque(float torque) {
    if (direction == FORWARD) {

    }

}

static float get_tape_speed() {

}
/*
static void ff_controller(float* u_t, float* u_s, float tape_speed) {
    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    //*u_t = filter_next(e_t, &controller_ff_takeup);
    *u_t = 0.0f;
    *u_s = filter_next(e_s, &controller_ff_supply);

    float e_tape_speed = tape_speed - control_state.tape_speed;
    float u_tape_speed = filter_next(e_tape_speed, &controller_tape_speed_ff);
    
    *u_t += -0.3f;
    //uart_println_float(u_tape_speed);

    //*u_t += u_tape_speed ;
}

static void rew_controller(float* u_t, float* u_s, float tape_speed) {
    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_rew_takeup);
    *u_s = filter_next(e_s, &controller_rew_supply);

    float e_tape_speed = tape_speed - control_state.tape_speed;
    float u_tape_speed = filter_next(e_tape_speed, &controller_tape_speed_rew);

    *u_s -= u_tape_speed ;
}
*/

