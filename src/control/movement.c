#include "movement.h"
#include "data_collector.h"
#include "filter.h"
#include "../sched.h"

static float integrator = 0.0f;
static MovementDirection direction = FORWARD;
static float time = 0.0f;
const float secondary_tension_reference = 0.6f;
const float primary_tension_reference = 1.2f;

// Function prototypes
static float simple_movement_command(float torque);
static float get_tape_speed();
static float get_primary_tension();
static float get_secondary_tension();
static ControlCommand get_motor_commands(MovementCommand movement_command);
static float init_controllers();

// Controllers
static Filter primary_tension_controller; 
static Filter secondary_tension_controller; 

// State
MovementState state;

static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);

void movement_init() {
    integrator = 0.0f;
    time = 0.0f;
    direction = FORWARD;
    state = IDLE;
    init_controllers();
}

static float init_controllers() {
    const float takeup_P = 1.0f;
    const float takeup_D = 0.05f;
    filter_init_pd(&primary_tension_controller, takeup_P, takeup_D, T);
}

static float simple_movement_command(float torque) {
    float primary_tension = get_primary_tension();
    float secondary_tension = get_secondary_tension();

    float error_primary_tension = primary_tension_reference - primary_tension;
    float error_secondary_tension = secondary_tension_reference - secondary_tension;
}

static float get_primary_tension() {
    if (direction == FORWARD) {
        return data_collector_get_takeup_tension();
    }
    return data_collector_get_supply_tension();
}

static float get_secondary_tension() {
    if (direction == FORWARD) {
        return data_collector_get_supply_tension();
    }
    return data_collector_get_takeup_tension();
}

static float get_tape_speed() {
    float tape_vel = data_collector_get_tape_speed();

    if (direction == FORWARD) {
        return tape_vel;
    }

    return -tape_vel;
}

static ControlCommand get_motor_commands(MovementCommand movement_command) {
    ControlCommand command;
    if (direction == FORWARD) {
        command.u_takeup = -movement_command.u_primary;
        command.u_supply = movement_command.u_secondary;
    } else {
        command.u_takeup = -movement_command.u_secondary;
        command.u_supply = movement_command.u_primary;
    }
    return command;
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

