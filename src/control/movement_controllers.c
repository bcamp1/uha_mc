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
} ControllerInfo;

static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);
static const float f = FREQUENCY_STATE_MACHINE_TICK;

static MovementCommand simple_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque);

TransitionStatus idle_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    const float transition_speed_thresh = 0.5f;
    const float transition_position_thresh = 0.25f;

    static float prev_tension_p = 0.0f;
    static float prev_tension_s = 0.0f;

    static Filter primary_filter;
    static Filter secondary_filter;

    if (info.ticks == 0) {
        filter_init_pd(&primary_filter, 1.0f, 0.05f, T);
        filter_init_pd(&secondary_filter, 1.0f, 0.05f, T);
    }
    
    float r = 0.5f;
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


    if (delta_primary < transition_speed_thresh && delta_secondary < transition_speed_thresh) {
        if (info.primary_tension > transition_position_thresh && info.secondary_tension > transition_position_thresh) {
            return TRANSITION_READY;
        }
    }

    return TRANSITION_NOT_READY;
}

TransitionStatus start_tension_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    const float transition_position_thresh = 0.95f;
    const float primary_slew_rate = 0.8; // Torque per second
    const float max_torque = 2.0f;
    float time = info.ticks * T;
    float torque = primary_slew_rate * time;
    if (torque > max_torque) torque = max_torque;
    idle_controller(info, target, command);
    command->u_primary += torque;
    if (info.primary_tension > transition_position_thresh) {
        return TRANSITION_READY;
    }
    return TRANSITION_NOT_READY;
}

TransitionStatus accelerate_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    static int init = 0;
    
    if (init == 0) {
        uart_println_int(info.ticks);
        init = 1;
    }


    MovementCommand c = simple_movement_command(info.ticks, info.primary_tension, info.secondary_tension, 0.4f);
    command->u_primary = c.u_primary;
    command->u_secondary = c.u_secondary;
    //uart_println_float(c.u_primary);
    return TRANSITION_NOT_READY;
}

TransitionStatus closed_loop_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    return TRANSITION_NOT_READY;
}

TransitionStatus decelerate_controller(ControllerInfo info, MovementTarget target, MovementCommand* command) {
    return TRANSITION_READY;
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

TransitionStatus movement_controllers_tick(MovementState state, MovementTarget target, uint32_t ticks, float t_p, float t_s, float tape_speed, MovementCommand* command) {
    ControllerInfo info = (ControllerInfo) {
        .ticks = ticks,
        .primary_tension = t_p,
        .secondary_tension = t_s,
        .tape_speed = tape_speed,
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

// This command assumes arms are properly tensioned already
static MovementCommand simple_movement_command(uint32_t ticks, float primary_tension, float secondary_tension, float torque) {
    static Filter secondary_filter;
    static Filter primary_filter;

    if (ticks == 0) {
        filter_init_pd(&primary_filter, 1.0f, 0.00f, T);
        filter_init_pd(&secondary_filter, 1.0f, 0.05f, T);
    }

    const float secondary_tension_reference = 0.6f;
    const float primary_tension_reference = 1.0f;
    float error_secondary_tension = secondary_tension_reference - secondary_tension;
    float error_primary_tension = primary_tension_reference - primary_tension;

    float u_primary = filter_next(error_primary_tension, &primary_filter) + torque;
    float u_secondary = filter_next(error_secondary_tension, &secondary_filter);

    return (MovementCommand) {
        .u_primary = u_primary,
        .u_secondary = u_secondary,
    };
}


