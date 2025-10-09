#include "state_machine.h"
#include "../drivers/tension_arm.h"
#include "../drivers/bldc.h"
#include "../drivers/delay.h"
#include "../drivers/stepper.h"
#include "../periphs/uart.h"

static State state = STOPPED;
static State next_state = STOPPED;
uint32_t current_state_ticks = 0;
bool is_closed_loop = true;

float takeup_speed = 0.0f;
float supply_speed = 0.0f;

static void playback_controller(float tension_t, float tension_s, float* u_t, float* u_s);
static void ff_controller(float tension_t, float tension_s, float* u_t, float* u_s);
static void rew_controller(float tension_t, float tension_s, float* u_t, float* u_s);
static void ff_to_stop_controller(float tension_t, float tension_s, float* u_t, float* u_s);
static void rew_to_stop_controller(float tension_t, float tension_s, float* u_t, float* u_s);
static void playback_to_stop_controller(float tension_t, float tension_s, float* u_t, float* u_s);

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
        case FF_TO_STOP:
            uart_println("FF_TO_STOP");
            break;
        case REW_TO_STOP:
            uart_println("REW_TO_STOP");
            break;
        case PLAYBACK_TO_STOP:
            uart_println("PLAYBACK_TO_STOP");
            break;
    }
}

void state_machine_init() {
    // Initialize necessary peripherals
    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);

    bldc_init_all();

    stepper_capstan_init();
    
    state = STOPPED;
    current_state_ticks = 0;
    is_closed_loop = true;
}

void state_machine_tick() {
    if (!is_closed_loop) {
        return;
    }

    float tension_t = tension_arm_get_position(&TENSION_ARM_A);
    float tension_s = tension_arm_get_position(&TENSION_ARM_B);

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
            playback_controller(tension_t, tension_s, &u_t, &u_s);
            break;
        case FF:
            ff_controller(tension_t, tension_s, &u_t, &u_s);
            break;
        case REW:
            rew_controller(tension_t, tension_s, &u_t, &u_s);
            break;
        case FF_TO_STOP:
            ff_to_stop_controller(tension_t, tension_s, &u_t, &u_s);
            break;
        case REW_TO_STOP:
            rew_to_stop_controller(tension_t, tension_s, &u_t, &u_s);
            break;
        case PLAYBACK_TO_STOP:
            playback_to_stop_controller(tension_t, tension_s, &u_t, &u_s);
            break;
    }

    takeup_speed = bldc_set_torque_float(&BLDC_CONF_TAKEUP, u_t);
    supply_speed = bldc_set_torque_float(&BLDC_CONF_SUPPLY, u_s);
    current_state_ticks++;
}

void state_machine_take_action(StateAction a) {
    is_closed_loop = false;
    switch (a) {
        case PLAY_ACTION:
            next_state = PLAYBACK;
            break;
        case STOP_ACTION:
            next_state = STOPPED;
            stepper_capstan_disengage();
            break;
        case FF_ACTION:
            next_state = FF;
            break;
        case REW_ACTION:
            next_state = REW;
            break;
    }

    if (state != next_state) {
        switch (state) {
            case STOPPED:
                switch(next_state) {
                    case FF:
                        stepper_capstan_disengage();
                        set_state(FF);
                        break;
                    case REW:
                        stepper_capstan_disengage();
                        set_state(REW);
                        break;
                    case PLAYBACK:
                        //stepper_capstan_engage();
                        set_state(PLAYBACK);
                        break;
                    default:
                        stepper_capstan_disengage();
                        set_state(STOPPED);
                        next_state = STOPPED;
                        break;
                }
                break;
            case FF:
                set_state(FF_TO_STOP);
                break;
            case REW:
                set_state(REW_TO_STOP);
                break;
            case PLAYBACK:
                set_state(PLAYBACK_TO_STOP);
                break;
            case FF_TO_STOP:
                break;
            case REW_TO_STOP:
                break;
            case PLAYBACK_TO_STOP:
                break;
        }
    }
    is_closed_loop = true;
}

static void playback_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = -0.8;
    const float k_s = 0.8;

    const float r_t = 0.5f;
    const float r_s = 0.5f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;
}

static void ff_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = 0.3;
    const float k_s = 0.8;

    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;

    *u_t -= 0.6f;
}

static void rew_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = -0.8;
    const float k_s = -0.3;

    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;

    *u_s += 0.6f;
}

static void ff_to_stop_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = -0.8;
    const float k_s = 0.8;

    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;

    if (current_state_ticks >= 1000) {
        set_state(STOPPED);
    }
}

static void rew_to_stop_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = -0.8;
    const float k_s = 0.8;

    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;

    if (current_state_ticks >= 1000) {
        set_state(STOPPED);
    }
}

static void playback_to_stop_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    set_state(STOPPED);
}

float state_machine_get_supply_speed() {
    return supply_speed;
}

float state_machine_get_takeup_speed() {
    return takeup_speed;
}
