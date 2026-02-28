#include "movement_controllers.h"
#include "../sched.h"
#include "../periphs/uart.h"
#include "filter.h"
#include "data_collector.h"

typedef struct {
    uint32_t ticks;
    float primary_tension;
    float secondary_tension;
    float tape_speed;
    float tape_position;
} ControllerInfo;

static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);
static const float f = FREQUENCY_STATE_MACHINE_TICK;

static MovementCommand accelerate_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque);
static MovementCommand decelerate_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque);

static float get_mem_distance(ControllerInfo info, MovementTarget target) {
    float delta = target.tape_position - info.tape_position;
    if (target.direction == FORWARD) {
        return delta;
    }
    return -delta;
}

TransitionStatus idle_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    const float transition_speed_thresh = 0.5f;
    const float transition_position_thresh = 0.25f;

    static float prev_tension_p = 0.0f;
    static float prev_tension_s = 0.0f;

    static Filter primary_filter;
    static Filter secondary_filter;

    if (info.ticks == 0) {
        filter_init_pd(&primary_filter, 0.0f, 0.00f, T);
        filter_init_pd(&secondary_filter, 0.0f, 0.00f, T);
    }
    
    float r = 0.6f;
    float e_primary = r - info.primary_tension;
    float e_secondary = r - info.secondary_tension;

    float u_primary = filter_next(e_primary, &primary_filter);
    float u_secondary = filter_next(e_secondary, &secondary_filter);

    command->u_primary = u_primary;
    command->u_secondary = u_secondary;

    float delta_primary = (info.primary_tension - prev_tension_p) * f;
    float delta_secondary = (info.secondary_tension - prev_tension_s) * f;

    if (delta_primary < 0) delta_primary = -delta_primary;
    if (delta_secondary < 0) delta_secondary = -delta_secondary;
    
    prev_tension_p = info.primary_tension;
    prev_tension_s = info.secondary_tension;
    
    float time = info.ticks * T;

    if (delta_primary < transition_speed_thresh && delta_secondary < transition_speed_thresh) {
        if (info.primary_tension > transition_position_thresh && info.secondary_tension > transition_position_thresh) {
            if (time > 0.25f) {
                return TRANSITION_READY;
            }
        }
    }

    return TRANSITION_NOT_READY;
}

TransitionStatus start_tension_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    const float transition_position_thresh = 0.95f;

    const float primary_slew_rate = 0.4f; // Torque per second
    const float max_torque = 2.0f;
    static float start_torque = 0.2f;
    const float r = 0.6f;

    static Filter primary_filter;
    static Filter secondary_filter;

    if (info.ticks == 0) {
        filter_init_pd(&primary_filter, 1.0f, 0.05f, T);
        filter_init_pd(&secondary_filter, 1.0f, 0.05f, T);
        //start_torque = filter_next(r - info.primary_tension, &primary_filter);
    }

    float time = info.ticks * T;
    float torque = (primary_slew_rate * time) + start_torque;
    if (torque > max_torque) torque = max_torque;

    float e_secondary = r - info.secondary_tension;
    float u_secondary = filter_next(e_secondary, &secondary_filter);

    command->u_secondary = u_secondary;
    command->u_primary = torque;

    if (info.primary_tension > transition_position_thresh) {
        return TRANSITION_READY;
    }
    return TRANSITION_NOT_READY;
}

TransitionStatus accelerate_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    static Filter tape_speed_controller;
    static float tape_speed_error_integrator;
    static float tape_speed_r;
    
    const float tape_speed_start = 10.0f;
    const float tape_speed_slew = 20.0f; // Units: IPS per second
    const float torque_floor = 0.1f;
    const float torque_ceil = 0.8f;
    const float PD_torque_ceil = 0.3f;
    const float tape_speed_target = 150.0f;
    const float Ki = 0.1f;

    if (info.ticks == 0) {
        filter_init_pd(&tape_speed_controller, 0.1f, 0.00f, T);
        tape_speed_error_integrator = 0.0f;
        tape_speed_r = 0.0f;
    }

    float time = info.ticks * T;
    tape_speed_r = tape_speed_slew * time;
    if (tape_speed_r > tape_speed_target) tape_speed_r = tape_speed_target;


    float tape_speed_e = tape_speed_r - info.tape_speed; 
    float torque_pd = filter_next(tape_speed_e, &tape_speed_controller);

    tape_speed_error_integrator += (tape_speed_e * T);
    float torque_i = Ki * tape_speed_error_integrator;

    if (torque_pd > PD_torque_ceil) torque_pd = PD_torque_ceil;
    
    float torque = torque_pd + torque_i;

    if (torque < torque_floor) torque = torque_floor;
    if (torque > torque_ceil) torque = torque_ceil;

    MovementCommand c = accelerate_movement_command(info.ticks, info.primary_tension, info.secondary_tension, torque);
    command->u_primary = c.u_primary;
    command->u_secondary = c.u_secondary;
    //uart_println_float(c.u_primary);
    
    // Decide when to transition to decelerate for mem option
    if (target.goto_position) {
        if (get_mem_distance(info, target) < 40.0f) {
            return TRANSITION_READY;
        }
    }
    return TRANSITION_NOT_READY;
}

TransitionStatus closed_loop_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    return TRANSITION_READY;
}

TransitionStatus decelerate_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    const float tape_speed_thresh = 0.8f;

    MovementCommand c = decelerate_movement_command(info.ticks, info.primary_tension, info.secondary_tension, 0.0f);
    command->u_primary = c.u_primary;
    command->u_secondary = c.u_secondary;

    if (info.tape_speed < tape_speed_thresh) {
        return TRANSITION_READY;
    }
    return TRANSITION_NOT_READY;
}

typedef enum {DEC_GOTO_MEDIUM, DEC_MEDIUM, DEC_GOTO_SLOW, DEC_SLOW} DecelerateMemState;

TransitionStatus decelerate_mem_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    static Filter tape_speed_controller;
    static float tape_speed_error_integrator;
    static float tape_speed_r;
    static float tape_speed_start = 0.0f;
    static DecelerateMemState dec_state = DEC_GOTO_MEDIUM;
    static uint32_t dec_state_ticks = 0;
    static float dec_state_time = 0.0f;

    const float torque_floor = -0.4f;
    const float torque_ceil = 0.4f;

    const float medium_tape_speed_target = 80.0f;
    const float slow_tape_speed_target = 15.0f;
    const float Ki = 0.1f;

    // Tape Distance thresholds
    const float distance_thresh_medium = 20.0f; // seconds
    const float distance_thresh_slow = 0.5f; // seconds

    // Slew rates
    const float slew_goto_medium = -20.0f; // IPS per second
    const float slew_goto_slow = -20.0f; // IPS per second

    if (info.ticks == 0) {
        tape_speed_start = info.tape_speed;
        filter_init_pd(&tape_speed_controller, 0.1f, 0.00f, T);
        tape_speed_error_integrator = 0.0f;
        tape_speed_r = 0.0f;
        dec_state_ticks = 0;
        dec_state_time = 0.0f;

        if (tape_speed_start > medium_tape_speed_target) {
            dec_state = DEC_GOTO_MEDIUM;
        } else {
            dec_state = DEC_GOTO_SLOW;
        }
    }

    // Calculate times
    float time = info.ticks * T;
    float distance = get_mem_distance(info, target);
    dec_state_time = dec_state_ticks * T;

    // Calculate tape speed reference + state transitions
    switch (dec_state) {
        case DEC_GOTO_MEDIUM:
            tape_speed_r = tape_speed_start + (dec_state_time * slew_goto_medium);
            if (tape_speed_r < medium_tape_speed_target) {
                tape_speed_r = medium_tape_speed_target;
                dec_state_ticks = 0;
                dec_state = DEC_MEDIUM;
            }
            break;
        case DEC_MEDIUM:
            tape_speed_r = medium_tape_speed_target;
            if (distance < distance_thresh_medium) {
                dec_state_ticks = 0;
                dec_state = DEC_GOTO_SLOW;
            }
            break;
        case DEC_GOTO_SLOW:
            tape_speed_r = medium_tape_speed_target + (dec_state_time * slew_goto_slow);
            if (tape_speed_r < slow_tape_speed_target) {
                tape_speed_r = slow_tape_speed_target;
                dec_state_ticks = 0;
                dec_state = DEC_SLOW;
            }
            break;
        case DEC_SLOW:
            tape_speed_r = slow_tape_speed_target;
            if (distance < distance_thresh_slow) {
               return TRANSITION_READY; 
            }
            break;
    }

    float tape_speed_e = tape_speed_r - info.tape_speed; 
    float torque_pd = filter_next(tape_speed_e, &tape_speed_controller);

    tape_speed_error_integrator += (tape_speed_e * T);
    float torque_i = Ki * tape_speed_error_integrator;
    
    float torque = torque_pd + torque_i;

    if (torque < torque_floor) torque = torque_floor;
    if (torque > torque_ceil) torque = torque_ceil;

    MovementCommand c = decelerate_movement_command(info.ticks, info.primary_tension, info.secondary_tension, torque);
    command->u_primary = c.u_primary;
    command->u_secondary = c.u_secondary;
    //uart_println_float(c.u_primary);

    dec_state_ticks++;
    return TRANSITION_NOT_READY;
}

TransitionStatus stop_tension_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    return TRANSITION_READY;
}

TransitionStatus playback_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    idle_controller(info, target, command);

    if (target.is_playback) {
        return TRANSITION_NOT_READY;
    }

    return TRANSITION_READY;
}

void movement_controllers_init() {

}

TransitionStatus movement_controllers_tick(MovementState state, MovementTarget target, uint32_t ticks, float t_p, float t_s, float tape_speed, float tape_position, MovementCommand* command) {
    ControllerInfo info = (ControllerInfo) {
        .ticks = ticks,
        .primary_tension = t_p,
        .secondary_tension = t_s,
        .tape_speed = tape_speed,
        .tape_position = tape_position,
    };

    switch (state) {
        case MV_IDLE:
            return idle_controller(info, target, command);
            break;
        case START_TENSION:
            return start_tension_controller(info, target, command);
            break;
        case ACCELERATE:
            return accelerate_controller(info, target, command);
            break;
        case CLOSED_LOOP:
            return closed_loop_controller(info, target, command);
            break;
        case DECELERATE:
            if (target.goto_position) {
                return decelerate_mem_controller(info, target, command);
            }
            return decelerate_controller(info, target, command);
            break;
        case STOP_TENSION:
            return stop_tension_controller(info, target, command);
            break;
        case MV_PLAYBACK:
            return playback_controller(info, target, command);
            break;
    }

    return TRANSITION_NOT_READY;
}

static MovementCommand accelerate_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque) {
    static Filter secondary_filter;
    static Filter primary_filter;

    if (ticks == 0) {
        filter_init_pd(&primary_filter, 1.0f, 0.05f, T);
        filter_init_pd(&secondary_filter, 1.0f, 0.05f, T);
    }

    const float secondary_tension_reference = 0.6f;
    const float primary_tension_reference = 1.1f;
    float error_secondary_tension = secondary_tension_reference - secondary_tension;
    float error_primary_tension = primary_tension_reference - primary_tension;

    float u_primary = filter_next(error_primary_tension, &primary_filter) + torque;
    float u_secondary = filter_next(error_secondary_tension, &secondary_filter);

    return (MovementCommand) {
        .u_primary = u_primary,
        .u_secondary = u_secondary,
    };
}

static MovementCommand decelerate_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque) {
    // Torque adjustment to be more similar to accelerate_movement_command
    torque -= 0.3f;

    static Filter secondary_filter;
    static Filter primary_filter;

    if (ticks == 0) {
        filter_init_pd(&primary_filter, 4.0f, 0.2f, T);
        filter_init_pd(&secondary_filter, 1.0f, 0.05f, T);
    }

    const float secondary_tension_reference = 0.6f;
    const float primary_tension_reference = 1.1f;
    float error_secondary_tension = secondary_tension_reference - secondary_tension;
    float error_primary_tension = primary_tension_reference - primary_tension;

    float u_primary = filter_next(error_primary_tension, &primary_filter) + torque;
    float u_secondary = filter_next(error_secondary_tension, &secondary_filter);

    return (MovementCommand) {
        .u_primary = u_primary,
        .u_secondary = u_secondary,
    };
}

