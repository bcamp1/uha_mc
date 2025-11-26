#include "state_machine.h"
#include "../drivers/tension_arm.h"
#include "../drivers/inc_encoder.h"
#include "../drivers/bldc.h"
#include "../drivers/delay.h"
#include "../drivers/solenoid.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "../board.h"
#include "filter.h"
#include "controllers.h"
#include "simple_filter.h"
#include <stdbool.h>

static State state = STOPPED;
static StateAction next_action = NO_ACTION;
uint32_t current_state_ticks = 0;
bool is_closed_loop = true;

static ControlState control_state;
static float tape_position_prev = 0.0f;

// Filters
static SimpleFilter tape_speed_filter;
static SimpleFilter takeup_tension_filter;
static SimpleFilter supply_tension_filter;

// Sample Period
static const float T = (1.0 / STATE_MACHINE_FREQUENCY);

// Reel Speeds (Not currently used)
static float takeup_speed;
static float supply_speed;

// Memory & Return to Zero 
static float mem_target_position = 0.0f;

static void init_filters();
static void playback_controller(float* u_t, float* u_s);
static void ff_controller(float* u_t, float* u_s, float torque);
static void rew_controller(float* u_t, float* u_s, float torque);
static void ff_to_idle_controller(float* u_t, float* u_s);
static void rew_to_idle_controller(float* u_t, float* u_s);
static void playback_to_idle_controller(float* u_t, float* u_s);
static void goto_mem_controller(float* u_t, float* u_s);
static bool idle_controller(float* u_t, float* u_s);

float state_machine_get_tape_speed() {
    return control_state.tape_speed;
}

static void init_filters() {
    tape_speed_filter = (SimpleFilter) {
        .alpha = 0.99f,
        .y_kminus1 = 0.0f,
    };
    takeup_tension_filter = (SimpleFilter) {
        .alpha = 0.9f,
        .y_kminus1 = 0.0f,
    };
    supply_tension_filter = (SimpleFilter) {
        .alpha = 0.9f,
        .y_kminus1 = 0.0f,
    };
}

static void set_state(State s) {
    state = s;
    current_state_ticks = 0;

    uart_print("[STATE] ");
    switch (state) {
        case STOPPED:
            uart_println("STOPPED");
            break;
        case PLAYBACK:
            uart_println("PLAYBACK");
            break;
        case FF:
            uart_println("FF");
            break;
        case REW:
            uart_println("REW");
            break;
        case FF_TO_IDLE:
            uart_println("FF_TO_STOP");
            break;
        case REW_TO_IDLE:
            uart_println("REW_TO_STOP");
            break;
        case PLAYBACK_TO_IDLE:
            uart_println("PLAYBACK_TO_IDLE");
            break;
        case IDLE:
            uart_println("IDLE");
            break;
        case GOTO_MEM:
            uart_println("GOTO_MEM");
            break;
    }
}

void state_machine_init() {
    // Initialize necessary peripherals
    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);

    solenoid_pinch_init();

    // Initialize tape speed filter
    init_filters();

    // Initialize Controllers
    controllers_init_all(T);

    bldc_init_all();
    
    state = STOPPED;
    current_state_ticks = 0;
    is_closed_loop = true;
}

void state_machine_tick() {

    gpio_set_pin(PIN_DEBUG1);
    if (!is_closed_loop) {
        return;
    }
    
    // New tensions
    float tension_t = tension_arm_get_position(&TENSION_ARM_A);
    float tension_s = tension_arm_get_position(&TENSION_ARM_B);
    float takeup_tension = simple_filter_next(tension_t, &takeup_tension_filter);
    float supply_tension = simple_filter_next(tension_s, &supply_tension_filter);
    
    // New tape speed
    tape_position_prev = control_state.tape_position;
    float tape_position = inc_encoder_get_position();
    float tape_delta = (tape_position - tape_position_prev) / T;
    float tape_speed = simple_filter_next(tape_delta, &tape_speed_filter);

    control_state.tape_position = tape_position;
    control_state.tape_speed = tape_speed;


    control_state.filtered_takeup_tension = takeup_tension;
    control_state.filtered_supply_tension = supply_tension;
    control_state.takeup_tension = tension_t;
    control_state.supply_tension = tension_s;

    float u_t = 0.0f;
    float u_s = 0.0f;

    if (state == STOPPED) {
        bldc_disable_all();
    } else {
        bldc_enable(&BLDC_CONF_SUPPLY);
        bldc_enable(&BLDC_CONF_TAKEUP);
    }

    if (state == PLAYBACK) {
        bldc_enable(&BLDC_CONF_CAPSTAN);
    } else {
        bldc_disable(&BLDC_CONF_CAPSTAN);
    }
    
    switch (state) {
        case STOPPED:
            break;
        case PLAYBACK:
            playback_controller(&u_t, &u_s);
            break;
        case FF:
            ff_controller(&u_t, &u_s, 0.5f);
            break;
        case REW:
            rew_controller(&u_t, &u_s, 0.5f);
            break;
        case FF_TO_IDLE:
            ff_to_idle_controller(&u_t, &u_s);
            break;
        case REW_TO_IDLE:
            rew_to_idle_controller(&u_t, &u_s);
            break;
        case PLAYBACK_TO_IDLE:
            playback_to_idle_controller(&u_t, &u_s);
            break;
        case IDLE:
            bool ready = idle_controller(&u_t, &u_s);
            if (ready) {
                state_machine_take_action(next_action);
                next_action = NO_ACTION;
            }
            break;
        case GOTO_MEM:
            goto_mem_controller(&u_t, &u_s);
            break;
    }

    takeup_speed = bldc_set_torque_float(&BLDC_CONF_TAKEUP, u_t);
    supply_speed = bldc_set_torque_float(&BLDC_CONF_SUPPLY, u_s);
    current_state_ticks++;
    gpio_clear_pin(PIN_DEBUG1);
}

void state_machine_take_action(StateAction a) {
    is_closed_loop = false;
    if (a == NO_ACTION) {
        return;
    }
    switch (state) {
        case STOPPED:
        case IDLE:
            switch (a) {
                case PLAY_ACTION:
                    bldc_enable(&BLDC_CONF_CAPSTAN);
                    set_state(PLAYBACK);
                    break;
                case STOP_ACTION:
                    solenoid_pinch_disengage();
                    set_state(STOPPED);
                    break;
                case FF_ACTION:
                    solenoid_pinch_disengage();
                    set_state(FF);
                    break;
                case REW_ACTION:
                    solenoid_pinch_disengage();
                    set_state(REW);
                    break;
                case MEM_ACTION:
                    solenoid_pinch_disengage();
                    set_state(GOTO_MEM);
                    break;
                case NO_ACTION:
                    break;
            }
            break;
        case FF:
            if (a != FF_ACTION) {
                set_state(FF_TO_IDLE);
                next_action = a;
            }
            break;
        case REW:
            if (a != REW_ACTION) {
                set_state(REW_TO_IDLE);
                next_action = a;
            }
            break;
        case PLAYBACK:
            if (a != PLAY_ACTION) {
                solenoid_pinch_disengage();
                set_state(PLAYBACK_TO_IDLE);
                next_action = a;
            }
            break;
        case GOTO_MEM:
            if (mem_target_position > control_state.tape_position) {
                set_state(FF_TO_IDLE);
                next_action = a;
            } else {
                set_state(REW_TO_IDLE);
                next_action = a;
            }
            break;
        case FF_TO_IDLE:
        case REW_TO_IDLE:
        case PLAYBACK_TO_IDLE:
            next_action = a;
            break;
    }
    is_closed_loop = true;
}

static void playback_controller(float* u_t, float* u_s) {
    const float r_t = 0.5f;
    const float r_s = 0.5f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_playback_takeup);
    *u_s = filter_next(e_s, &controller_playback_supply);

    if (current_state_ticks == 500) {
        solenoid_pinch_engage();
    }
}

static void ff_controller(float* u_t, float* u_s, float torque) {
    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_ff_takeup);
    *u_s = filter_next(e_s, &controller_ff_supply);

    *u_t -= torque;
}

static void rew_controller(float* u_t, float* u_s, float torque) {
    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_rew_takeup);
    *u_s = filter_next(e_s, &controller_rew_supply);

    *u_s += torque;
}

static void ff_to_idle_controller(float* u_t, float* u_s) {
    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_ff_takeup);
    *u_s = filter_next(e_s, &controller_ff_supply);

    if (current_state_ticks >= 1000) {
        set_state(IDLE);
    }
}

static void rew_to_idle_controller(float* u_t, float* u_s) {
    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_rew_takeup);
    *u_s = filter_next(e_s, &controller_rew_supply);

    if (current_state_ticks >= 1000) {
        set_state(IDLE);
    }
}

static void playback_to_idle_controller(float* u_t, float* u_s) {
    const float r_t = 0.5f;
    const float r_s = 0.5f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_idle_takeup);
    *u_s = filter_next(e_s, &controller_idle_supply);

    if (current_state_ticks >= 500) {
        set_state(IDLE);
    }
}

static bool idle_controller(float* u_t, float* u_s) {
    const float r_t = 0.5f;
    const float r_s = 0.5f;

    float e_t = r_t - control_state.filtered_takeup_tension;
    float e_s = r_s - control_state.filtered_supply_tension;

    *u_t = filter_next(e_t, &controller_idle_takeup);
    *u_s = filter_next(e_s, &controller_idle_supply);

    const float reel_speed_thresh = 0.5;

    // Check if ready for next action
    if (supply_speed < reel_speed_thresh && supply_speed > -reel_speed_thresh) {
        if (takeup_speed < reel_speed_thresh && takeup_speed > -reel_speed_thresh) {
            return true;
            if (e_t < 0.2f && e_t > -0.2f) {
                if (e_s < 0.2f && e_s > -0.2f) {
                    return true;
                }
            }
        }
    }
    return false;
}

static void goto_mem_controller(float* u_t, float* u_s) {
    const float full_gain_distance = 60.0f;
    const float max_torque = 0.6f;
    const float min_torque = 0.4f;
    const float error_threshold = 0.5f;

    float gain = 1.0f / full_gain_distance;

    float current_pos = control_state.tape_position; 
    float position_error = mem_target_position - current_pos;

    if (position_error < error_threshold && position_error > -error_threshold) {
        if (position_error > 0) {
            set_state(FF_TO_IDLE);
            next_action = STOP_ACTION;
        } else {
            set_state(FF_TO_IDLE);
            next_action = STOP_ACTION;
        }
        return;
    }

    float gain_percent = position_error * gain;
    bool is_ff = (gain_percent > 0);

    if (!is_ff) gain_percent = -gain_percent;
    if (gain_percent > 1.0f) gain_percent = 1.0f;

    float torque = min_torque + gain_percent*(max_torque - min_torque);

    //uart_println_float(torque);
    
    if (is_ff) {
        ff_controller(u_t, u_s, torque);
    } else {
        rew_controller(u_t, u_s, torque);
    }
}

float state_machine_get_supply_speed() {
    return supply_speed;
}

float state_machine_get_takeup_speed() {
    return takeup_speed;
}

void state_machine_goto_position(float position) {
    mem_target_position = position;
    state_machine_take_action(MEM_ACTION);
}

