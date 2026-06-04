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

// Shared state for the idle tension-regulation loop. Each caller (idle,
// playback) owns its own instance so the two modes can't alias each other's
// filters or tension history.
typedef struct {
    Filter primary_filter;
    Filter secondary_filter;
    float prev_tension_p;
    float prev_tension_s;
} IdleState;

// A pair of tension-regulation filters (primary/secondary reel). Used by the
// accelerate/decelerate feed-forward command helpers.
typedef struct {
    Filter primary_filter;
    Filter secondary_filter;
} TensionFilters;

static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);
static const float f = FREQUENCY_STATE_MACHINE_TICK;

static MovementCommand accelerate_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque);
static MovementCommand decelerate_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque);

// Only meaningful for a SEEK target (callers gate on TARGET_SEEK).
static float get_mem_distance(ControllerInfo info, MovementTarget target) {
    float delta = target.u.seek.position - info.tape_position;
    if (target.u.seek.dir == FORWARD) {
        return delta;
    }
    return -delta;
}

static TransitionStatus idle_regulate(ControllerInfo info, MovementCommand* command, IdleState* st) {
    const float transition_speed_thresh = 0.5f;
    const float transition_position_thresh = 0.25f;

    if (info.ticks == 0) {
        filter_init_pd(&st->primary_filter, 1.0f, 0.05f, T);
        filter_init_pd(&st->secondary_filter, 1.0f, 0.05f, T);
    }

    float r = 0.6f;
    float e_primary = r - info.primary_tension;
    float e_secondary = r - info.secondary_tension;

    float u_primary = filter_next(e_primary, &st->primary_filter);
    float u_secondary = filter_next(e_secondary, &st->secondary_filter);

    command->u_primary = u_primary;
    command->u_secondary = u_secondary;

    float delta_primary = (info.primary_tension - st->prev_tension_p) * f;
    float delta_secondary = (info.secondary_tension - st->prev_tension_s) * f;

    if (delta_primary < 0) delta_primary = -delta_primary;
    if (delta_secondary < 0) delta_secondary = -delta_secondary;

    st->prev_tension_p = info.primary_tension;
    st->prev_tension_s = info.secondary_tension;

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

TransitionStatus idle_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    static IdleState st;
    return idle_regulate(info, command, &st);
}

TransitionStatus start_tension_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    const float transition_position_thresh = 0.95f;

    const float primary_slew_rate = 0.4f; // Torque per second
    const float max_torque = 2.0f;
    static float start_torque = 0.2f;
    const float r = 0.6f;

    static Filter secondary_filter;

    if (info.ticks == 0) {
        filter_init_pd(&secondary_filter, 1.0f, 0.05f, T);
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

typedef struct {
    Filter tape_speed_controller;
    float tape_speed_error_integrator;
    float tape_speed_r;
} AccelerateState;

TransitionStatus accelerate_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    static AccelerateState st;

    const float tape_speed_start = 10.0f;
    const float tape_speed_slew = 20.0f; // Units: IPS per second
    const float torque_floor = 0.1f;
    const float torque_ceil = 0.8f;
    const float PD_torque_ceil = 0.3f;
    const float tape_speed_target = 150.0f;
    const float Ki = 0.1f;

    if (info.ticks == 0) {
        filter_init_pd(&st.tape_speed_controller, 0.1f, 0.00f, T);
        st.tape_speed_error_integrator = 0.0f;
        st.tape_speed_r = 0.0f;
    }

    float time = info.ticks * T;
    st.tape_speed_r = tape_speed_slew * time;
    if (st.tape_speed_r > tape_speed_target) st.tape_speed_r = tape_speed_target;


    float tape_speed_e = st.tape_speed_r - info.tape_speed;
    float torque_pd = filter_next(tape_speed_e, &st.tape_speed_controller);

    st.tape_speed_error_integrator += (tape_speed_e * T);
    float torque_i = Ki * st.tape_speed_error_integrator;

    if (torque_pd > PD_torque_ceil) torque_pd = PD_torque_ceil;
    
    float torque = torque_pd + torque_i;

    if (torque < torque_floor) torque = torque_floor;
    if (torque > torque_ceil) torque = torque_ceil;

    MovementCommand c = accelerate_movement_command(info.ticks, info.primary_tension, info.secondary_tension, torque);
    command->u_primary = c.u_primary;
    command->u_secondary = c.u_secondary;
    //uart_println_float(c.u_primary);
    
    // Decide when to transition to decelerate for mem option
    if (target.kind == TARGET_SEEK) {
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

typedef struct {
    Filter tape_speed_controller;
    float tape_speed_error_integrator;
    float tape_speed_r;
    float tape_speed_start;
    DecelerateMemState dec_state;
    uint32_t dec_state_ticks;
    float dec_state_time;
} DecelerateMemRunState;

TransitionStatus decelerate_mem_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    static DecelerateMemRunState st;

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
        st.tape_speed_start = info.tape_speed;
        filter_init_pd(&st.tape_speed_controller, 0.1f, 0.00f, T);
        st.tape_speed_error_integrator = 0.0f;
        st.tape_speed_r = 0.0f;
        st.dec_state_ticks = 0;
        st.dec_state_time = 0.0f;

        if (st.tape_speed_start > medium_tape_speed_target) {
            st.dec_state = DEC_GOTO_MEDIUM;
        } else {
            st.dec_state = DEC_GOTO_SLOW;
        }
    }

    // Calculate times
    float time = info.ticks * T;
    float distance = get_mem_distance(info, target);
    st.dec_state_time = st.dec_state_ticks * T;

    // Calculate tape speed reference + state transitions
    switch (st.dec_state) {
        case DEC_GOTO_MEDIUM:
            st.tape_speed_r = st.tape_speed_start + (st.dec_state_time * slew_goto_medium);
            if (st.tape_speed_r < medium_tape_speed_target) {
                st.tape_speed_r = medium_tape_speed_target;
                st.dec_state_ticks = 0;
                st.dec_state = DEC_MEDIUM;
            }
            break;
        case DEC_MEDIUM:
            st.tape_speed_r = medium_tape_speed_target;
            if (distance < distance_thresh_medium) {
                st.dec_state_ticks = 0;
                st.dec_state = DEC_GOTO_SLOW;
            }
            break;
        case DEC_GOTO_SLOW:
            st.tape_speed_r = medium_tape_speed_target + (st.dec_state_time * slew_goto_slow);
            if (st.tape_speed_r < slow_tape_speed_target) {
                st.tape_speed_r = slow_tape_speed_target;
                st.dec_state_ticks = 0;
                st.dec_state = DEC_SLOW;
            }
            break;
        case DEC_SLOW:
            st.tape_speed_r = slow_tape_speed_target;
            if (distance < distance_thresh_slow) {
               return TRANSITION_READY;
            }
            break;
    }

    float tape_speed_e = st.tape_speed_r - info.tape_speed;
    float torque_pd = filter_next(tape_speed_e, &st.tape_speed_controller);

    st.tape_speed_error_integrator += (tape_speed_e * T);
    float torque_i = Ki * st.tape_speed_error_integrator;

    float torque = torque_pd + torque_i;

    if (torque < torque_floor) torque = torque_floor;
    if (torque > torque_ceil) torque = torque_ceil;

    MovementCommand c = decelerate_movement_command(info.ticks, info.primary_tension, info.secondary_tension, torque);
    command->u_primary = c.u_primary;
    command->u_secondary = c.u_secondary;
    //uart_println_float(c.u_primary);

    st.dec_state_ticks++;
    return TRANSITION_NOT_READY;
}

TransitionStatus stop_tension_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    return TRANSITION_READY;
}

TransitionStatus playback_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    static IdleState st;
    idle_regulate(info, command, &st);

    if (target.kind == TARGET_PLAYBACK) {
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
            if (target.kind == TARGET_SEEK) {
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
    static TensionFilters st;

    if (ticks == 0) {
        filter_init_pd(&st.primary_filter, 1.0f, 0.05f, T);
        filter_init_pd(&st.secondary_filter, 1.0f, 0.05f, T);
    }

    const float secondary_tension_reference = 0.6f;
    const float primary_tension_reference = 1.1f;
    float error_secondary_tension = secondary_tension_reference - secondary_tension;
    float error_primary_tension = primary_tension_reference - primary_tension;

    float u_primary = filter_next(error_primary_tension, &st.primary_filter) + torque;
    float u_secondary = filter_next(error_secondary_tension, &st.secondary_filter);

    return (MovementCommand) {
        .u_primary = u_primary,
        .u_secondary = u_secondary,
    };
}

static MovementCommand decelerate_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque) {
    // Torque adjustment to be more similar to accelerate_movement_command
    torque -= 0.3f;

    static TensionFilters st;

    if (ticks == 0) {
        filter_init_pd(&st.primary_filter, 4.0f, 0.2f, T);
        filter_init_pd(&st.secondary_filter, 1.0f, 0.05f, T);
    }

    const float secondary_tension_reference = 0.6f;
    const float primary_tension_reference = 1.1f;
    float error_secondary_tension = secondary_tension_reference - secondary_tension;
    float error_primary_tension = primary_tension_reference - primary_tension;

    float u_primary = filter_next(error_primary_tension, &st.primary_filter) + torque;
    float u_secondary = filter_next(error_secondary_tension, &st.secondary_filter);

    return (MovementCommand) {
        .u_primary = u_primary,
        .u_secondary = u_secondary,
    };
}

