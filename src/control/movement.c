#include "movement.h"
#include "movement_controllers.h"
#include "data_collector.h"
#include "filter.h"
#include "../sched.h"
#include <stdint.h>
#include "sam.h"
#include "../periphs/uart.h"
#include "../drivers/bldc.h"
#include "../drivers/solenoid.h"

static float integrator = 0.0f;
static uint32_t ticks = 0;

// Function prototypes
static MovementCommand simple_movement_command(float torque);
static float get_tape_speed();
static float get_primary_tension();
static float get_secondary_tension();
static MotorCommand get_motor_commands(MovementCommand movement_command);
static void set_commanded_target(MovementTarget t);
static void transfer_commanded_target_current();
static void transfer_commanded_target_future();
static void transfer_future_target_current();
static void state_transition_commanded_target();
static void panic(char* str);
static void print_state(MovementState s);
static void set_state(MovementState s);

// Targets
static volatile MovementTarget target;
static volatile MovementTarget future_target;
static volatile MovementTarget commanded_target;

// State
static volatile MovementState state;

static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);

void movement_init() {
    integrator = 0.0f;
    ticks = -1;
    state = MV_IDLE;
    target.active = false;
    future_target.active = false;
    commanded_target.active = false;
    movement_set_target_idle();
    movement_controllers_init();
    bldc_enable(&BLDC_CONF_SUPPLY);
    bldc_enable(&BLDC_CONF_TAKEUP);
    bldc_set_torque_float(&BLDC_CONF_TAKEUP, 0.0f);
    bldc_set_torque_float(&BLDC_CONF_SUPPLY, 0.0f);
}

void movement_tick() {
    ticks += 1;

    // Step 0: Update data
    data_collector_update();

    // Step 1: Check for new target
    if (commanded_target.active) {
        state_transition_commanded_target(); 
    }

    // Step 2: Check if we need to switch to the future target
    if (!target.active) {
        if (state != MV_IDLE) {
           panic("Inactive non-idle state"); 
        } 

        if (future_target.active) {
            transfer_future_target_current();
        }
    }

    // Step 3: Execute current state's controller function and check for state switch
    MovementCommand command;
    TransitionStatus status = movement_controllers_tick(
        state,
        target,
        ticks,
        get_primary_tension(),
        get_secondary_tension(),
        get_tape_speed(),
        &command);

    // Step 4: Transition states if they are ready to transition
    if (status == TRANSITION_READY) {
        uart_println("ready");
        switch (state) {
            case MV_IDLE:
                if (!target.is_idle) {
                    if (target.active && target.is_playback) {
                        set_state(MV_PLAYBACK);
                    } else {
                        set_state(START_TENSION);
                    }
                }
                break;
            case START_TENSION:
                set_state(ACCELERATE);
                break;
            case ACCELERATE:
                set_state(CLOSED_LOOP);
                break;
            case CLOSED_LOOP:
                set_state(DECELERATE);
                break;
            case DECELERATE:
                set_state(STOP_TENSION);
                break;
            case STOP_TENSION:
                set_state(MV_IDLE);
                target.active = false;
                break;
            case MV_PLAYBACK:
                set_state(MV_IDLE);
                target.active = false;
                break;
        }
    }

    // Step 5: Apply command to motors
    MotorCommand motor_command = get_motor_commands(command);

    float takeup_speed = bldc_set_torque_float(&BLDC_CONF_TAKEUP, motor_command.u_takeup);
    float supply_speed = bldc_set_torque_float(&BLDC_CONF_SUPPLY, motor_command.u_supply);

    //uart_println_float(motor_command.u_takeup);
    
    data_collector_set_reel_speeds(takeup_speed, supply_speed);

}

static void state_transition_commanded_target() {
    if (commanded_target.direction != target.direction) {
        switch (state) {
            case MV_IDLE:
                transfer_commanded_target_current();
                break;
            case START_TENSION:
                set_state(STOP_TENSION);
                transfer_commanded_target_future();
                break;
            case ACCELERATE:
                set_state(DECELERATE);
                transfer_commanded_target_future();
                break;
            case CLOSED_LOOP:
                set_state(DECELERATE);
                transfer_commanded_target_future();
                break;
            case DECELERATE:
                transfer_commanded_target_future();
                break;
            case STOP_TENSION:
                transfer_commanded_target_future();
                break;
            case MV_PLAYBACK:
                set_state(MV_IDLE);
                transfer_commanded_target_current();
                break;
        }
    } else {
        switch (state) {
            // TODO: Smoother state transistions here
            case MV_IDLE:
                transfer_commanded_target_current();
                break;
            case START_TENSION:
                set_state(STOP_TENSION);
                transfer_commanded_target_future();
                break;
            case ACCELERATE:
                set_state(DECELERATE);
                transfer_commanded_target_future();
                break;
            case CLOSED_LOOP:
                set_state(DECELERATE);
                transfer_commanded_target_future();
                break;
            case DECELERATE:
                transfer_commanded_target_future();
                break;
            case STOP_TENSION:
                transfer_commanded_target_future();
                break;
            case MV_PLAYBACK:
                set_state(MV_IDLE);
                transfer_commanded_target_current();
                break;
        }
    }
}

void movement_set_target_ff(float tape_speed) {
    if (tape_speed < 0.0f) return;
    MovementTarget t = (MovementTarget) {
        .direction = FORWARD,
        .tape_speed = tape_speed,
        .tape_position = 0.0f,
        .goto_position = false,
        .is_playback = false,
        .is_idle = false,
        .active = true,
    }; 
    set_commanded_target(t);
}

void movement_set_target_rew(float tape_speed) {
    if (tape_speed < 0.0f) return;
    MovementTarget t = (MovementTarget) {
        .direction = REVERSE,
        .tape_speed = tape_speed,
        .tape_position = 0.0f,
        .goto_position = false,
        .is_playback = false,
        .is_idle = false,
        .active = true,
    }; 
    set_commanded_target(t);
}

void movement_set_target_mem(float tape_position, float tape_speed) {
    if (tape_speed < 0.0f) return;
    float current_position = data_collector_get_tape_position();

    MovementDirection dir;
    if (current_position < tape_position) {
        dir = FORWARD;
    } else {
        dir = REVERSE;
    }

    MovementTarget t = (MovementTarget) {
        .direction = dir,
        .tape_speed = tape_speed,
        .tape_position = tape_position,
        .goto_position = true,
        .is_playback = false,
        .is_idle = false,
        .active = true,
    }; 
    set_commanded_target(t);
}

void movement_set_target_playback() {
    MovementTarget t = (MovementTarget) {
        .direction = FORWARD,
        .tape_speed = 15.0f,
        .tape_position = 0.0f,
        .goto_position = false,
        .is_playback = true,
        .is_idle = false,
        .active = true,
    }; 
    set_commanded_target(t);
}

void movement_set_target_idle() {
    MovementTarget t = (MovementTarget) {
        .direction = FORWARD,
        .tape_speed = 0.0f,
        .tape_position = 0.0f,
        .goto_position = false,
        .is_playback = false,
        .is_idle = true,
        .active = true,
    }; 
    set_commanded_target(t);
}

static void set_commanded_target(MovementTarget t) {
    uint32_t primask = __get_PRIMASK();  // Save interrupt state
    __disable_irq();                     // Disable all maskable interrupts
    commanded_target = t;
    commanded_target.active = true;
    __set_PRIMASK(primask);              // Restore old interrupt state
}

static void transfer_commanded_target_current() {
    uint32_t primask = __get_PRIMASK();  // Save interrupt state
    __disable_irq();                     // Disable all maskable interrupts
    target = commanded_target;
    commanded_target.active = false;
    target.active = true;
    future_target.active = false;
    __set_PRIMASK(primask);              // Restore old interrupt state
    uart_println("target <- command");
}

static void transfer_commanded_target_future() {
    uint32_t primask = __get_PRIMASK();  // Save interrupt state
    __disable_irq();                     // Disable all maskable interrupts
    if (commanded_target.active) {
        future_target = commanded_target;
        commanded_target.active = false;
        uart_println("future target <- command");
    }
    __set_PRIMASK(primask);              // Restore old interrupt state
}

static void transfer_future_target_current() {
    uint32_t primask = __get_PRIMASK();  // Save interrupt state
    __disable_irq();                     // Disable all maskable interrupts
    target = future_target;
    __set_PRIMASK(primask);              // Restore old interrupt state
    uart_println("target <- future target");
}

static float get_primary_tension() {
    if (target.direction == FORWARD) {
        return data_collector_get_takeup_tension();
    }
    return data_collector_get_supply_tension();
}

static float get_secondary_tension() {
    if (target.direction == FORWARD) {
        return data_collector_get_supply_tension();
    }
    return data_collector_get_takeup_tension();
}

static float get_tape_speed() {
    float tape_vel = data_collector_get_tape_speed();

    if (target.direction == FORWARD) {
        return tape_vel;
    }

    return -tape_vel;
}

static MotorCommand get_motor_commands(MovementCommand movement_command) {
    MotorCommand command;
    if (target.direction == FORWARD) {
        command.u_takeup = -movement_command.u_primary;
        command.u_supply = movement_command.u_secondary;
    } else {
        command.u_takeup = -movement_command.u_secondary;
        command.u_supply = movement_command.u_primary;
    }
    return command;
}

static void panic(char* str) {
    __disable_irq();
        uart_print("ERR: ");
        uart_println(str);
        while (true) {}
}

static void set_state(MovementState s) {
    if (state == s) {
        panic("Attempted to set state but already current state");
    }
    state = s;
    print_state(s);
    ticks = -1;
    if (state == MV_PLAYBACK) {
        bldc_enable(&BLDC_CONF_CAPSTAN);
        solenoid_pinch_engage();
    } else {
        bldc_disable(&BLDC_CONF_CAPSTAN);
        solenoid_pinch_disengage();
    }
}

static void print_state(MovementState s) {
    uart_print("[MOVSTATE] ");
    switch (state) {
        case MV_IDLE:
            uart_println("MV_IDLE");
            break;
        case MV_PLAYBACK:
            uart_println("MV_PLAYBACK");
            break;
        case START_TENSION:
            uart_println("START_TENSION");
            break;
        case ACCELERATE:
            uart_println("ACCELERATE");
            break;
        case CLOSED_LOOP:
            uart_println("CLOSED_LOOP");
            break;
        case DECELERATE:
            uart_println("DECELERATE");
            break;
        case STOP_TENSION:
            uart_println("STOP_TENSION");
            break;
    }
}

