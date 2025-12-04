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
#include "../sched.h"
#include "data_collector.h"

static State state = STOPPED;
static StateAction next_action = NO_ACTION;
uint32_t current_state_ticks = 0;
bool is_closed_loop = true;

// Sample Period
static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);

// Memory & Return to Zero 
static float mem_target_position = 0.0f;

static void playback_controller(float* u_t, float* u_s);
static void ff_controller(float* u_t, float* u_s, float tape_speed);
static void rew_controller(float* u_t, float* u_s, float tape_speed);
static void ff_to_idle_controller(float* u_t, float* u_s);
static void rew_to_idle_controller(float* u_t, float* u_s);
static void playback_to_idle_controller(float* u_t, float* u_s);
static void goto_mem_controller(float* u_t, float* u_s);
static bool idle_controller(float* u_t, float* u_s);

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
    
    // Initialize data collection
    data_collector_init();

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
   
    data_collector_update();

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
            ff_controller(&u_t, &u_s, 1.0f);
            break;
        case REW:
            rew_controller(&u_t, &u_s, -0.5f);
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

    float takeup_speed = bldc_set_torque_float(&BLDC_CONF_TAKEUP, u_t);
    float supply_speed = bldc_set_torque_float(&BLDC_CONF_SUPPLY, u_s);
    
    data_collector_set_reel_speeds(takeup_speed, supply_speed);

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
            /*
            if (mem_target_position > control_state.tape_position) {
                set_state(FF_TO_IDLE);
                next_action = a;
            } else {
                set_state(REW_TO_IDLE);
                next_action = a;
            }
            */
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

    float e_t = r_t - data_collector_get_takeup_tension();
    float e_s = r_s - data_collector_get_supply_tension();

    *u_t = filter_next(e_t, &controller_playback_takeup);
    *u_s = filter_next(e_s, &controller_playback_supply);

    if (current_state_ticks == 500) {
        solenoid_pinch_engage();
    }
}

static void ff_controller(float* u_t, float* u_s, float tape_speed) {
    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_t = r_t - data_collector_get_takeup_tension();
    float e_s = r_s - data_collector_get_supply_tension();

    //*u_t = filter_next(e_t, &controller_ff_takeup);
    *u_t = 0.0f;
    *u_s = filter_next(e_s, &controller_ff_supply);

    float e_tape_speed = tape_speed - data_collector_get_tape_speed();
    float u_tape_speed = filter_next(e_tape_speed, &controller_tape_speed_ff);
    
    *u_t += -0.3f;
    //uart_println_float(u_tape_speed);

    //*u_t += u_tape_speed ;
}

static void rew_controller(float* u_t, float* u_s, float tape_speed) {
    /*
    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_t = r_t - data_collector_get_takeup_tension();
    float e_s = r_s - data_collector_get_supply_tension();

    *u_t = filter_next(e_t, &controller_rew_takeup);
    *u_s = filter_next(e_s, &controller_rew_supply);

    float e_tape_speed = tape_speed - control_state.tape_speed;
    float u_tape_speed = filter_next(e_tape_speed, &controller_tape_speed_rew);

    *u_s -= u_tape_speed ;
    */
}

static void ff_to_idle_controller(float* u_t, float* u_s) {
    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_t = r_t - data_collector_get_takeup_tension();
    float e_s = r_s - data_collector_get_supply_tension();

    *u_t = filter_next(e_t, &controller_ff_takeup);
    *u_s = filter_next(e_s, &controller_ff_supply);

    if (current_state_ticks >= 1000) {
        set_state(IDLE);
    }
}

static void rew_to_idle_controller(float* u_t, float* u_s) {
    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_t = r_t - data_collector_get_takeup_tension();
    float e_s = r_s - data_collector_get_supply_tension();

    *u_t = filter_next(e_t, &controller_rew_takeup);
    *u_s = filter_next(e_s, &controller_rew_supply);

    if (current_state_ticks >= 1000) {
        set_state(IDLE);
    }
}

static void playback_to_idle_controller(float* u_t, float* u_s) {
    const float r_t = 0.5f;
    const float r_s = 0.5f;

    float e_t = r_t - data_collector_get_takeup_tension();
    float e_s = r_s - data_collector_get_supply_tension();

    *u_t = filter_next(e_t, &controller_idle_takeup);
    *u_s = filter_next(e_s, &controller_idle_supply);

    if (current_state_ticks >= 500) {
        set_state(IDLE);
    }
}

static bool idle_controller(float* u_t, float* u_s) {
    const float r_t = 0.5f;
    const float r_s = 0.5f;

    float e_t = r_t - data_collector_get_takeup_tension();
    float e_s = r_s - data_collector_get_supply_tension();

    *u_t = filter_next(e_t, &controller_idle_takeup);
    *u_s = filter_next(e_s, &controller_idle_supply);

    const float reel_speed_thresh = 0.5;

    float supply_speed = data_collector_get_supply_reel_speed();
    float takeup_speed = data_collector_get_takeup_reel_speed();

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
    /*
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
    */
}

void state_machine_goto_position(float position) {
    mem_target_position = position;
    state_machine_take_action(MEM_ACTION);
}

