#include "movement.h"
#include "movement_controllers.h"
#include "data_collector.h"
#include "filter.h"
#include "../sched.h"
#include "../board.h"
#include "../drivers/motors.h"
#include <stdint.h>
#include "sam.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
//#include "../drivers/bldc.h"
#include "../drivers/solenoid.h"
#include "../drivers/stat_tracker.h"
#include <stdbool.h>

static float integrator = 0.0f;
static uint32_t ticks = 0;

// Odometer Stats
static volatile uint32_t playback_ticks = 0;
static volatile float prev_tape_position = 0.0f;
static volatile float playback_travel = 0.0f;

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
static void set_state(MovementState s);

// Targets
static volatile MovementTarget target;
static volatile MovementTarget future_target;
static volatile MovementTarget commanded_target;

// State
static volatile MovementState state;

// Set by the main-loop fault supervisor (movement_set_fault_disarm). While true,
// the tick stops driving the motor bus -- see the gate in movement_tick.
static volatile bool fault_disarmed = false;

// Optional auto-play: when true, a completed SEEK enters MV_PLAYBACK instead of
// stopping at MV_IDLE. Toggled only by UCOMM_M_SET/CLEAR_AUTO_PLAY; persists
// across STOP/DISABLE (movement_init does not touch it). Default off at boot.
static volatile bool auto_play = false;

static const float T = (1.0 / FREQUENCY_STATE_MACHINE_TICK);

// Enter a critical section by disabling interrupts, returning the previous
// PRIMASK so the caller can restore it. Safe to nest: if interrupts were
// already disabled, critical_exit() leaves them disabled.
static inline uint32_t critical_enter(void) {
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static inline void critical_exit(uint32_t primask) {
    __set_PRIMASK(primask);
}

// ---------------------------------------------------------------------------
// State transition tables
//
// A state's transition behavior lives in two places, both keyed by the current
// MovementState so they read side-by-side:
//
//   NEXT_ON_READY  - where to advance when the controller reports it is done
//                    (TRANSITION_READY). This is the normal forward pipeline.
//                    MV_IDLE is special-cased (its successor depends on the
//                    pending target) and never read from this table.
//
//   CMD_TRANSITION - how to react when a new command arrives mid-motion.
// ---------------------------------------------------------------------------

// Forward pipeline. Advancing to MV_IDLE means the run finished and the target
// slot is released (see movement_tick).
static const MovementState NEXT_ON_READY[] = {
    [START_TENSION] = ACCELERATE,
    [ACCELERATE]    = CLOSED_LOOP,
    [CLOSED_LOOP]   = DECELERATE,
    [DECELERATE]    = STOP_TENSION,
    [STOP_TENSION]  = MV_IDLE,
    [MV_PLAYBACK]   = MV_IDLE,
};

typedef struct {
    bool          set_next;      // change to `next` before loading the command?
    MovementState next;
    bool          abandon_seek;  // drop a SEEK position goal (decelerate normally)?
    bool          load_now;      // true: take command as current target now;
                                 // false: stash it as the future target
} CommandTransition;

static const CommandTransition CMD_TRANSITION[] = {
    [MV_IDLE]       = { .load_now = true },
    [START_TENSION] = { .set_next = true, .next = STOP_TENSION },
    [ACCELERATE]    = { .set_next = true, .next = DECELERATE, .abandon_seek = true },
    [CLOSED_LOOP]   = { .set_next = true, .next = DECELERATE, .abandon_seek = true },
    [DECELERATE]    = { .abandon_seek = true },
    [STOP_TENSION]  = { 0 },  // just stash the command as the future target
    [MV_PLAYBACK]   = { .set_next = true, .next = MV_IDLE, .load_now = true },
};

float movement_get_playback_time() {
    float ticks = (float) playback_ticks;
    return ticks / FREQUENCY_STATE_MACHINE_TICK;
}

float movement_get_playback_travel() {
    return playback_travel;
}

// Restore the cumulative odometer counters from persistent storage. Done once,
// on the first movement_init() (boot): movement_init() also runs on every
// STOP/disarm, and the stat flusher only persists every ~5 s, so reloading on
// later calls would roll the totals back to the last-flushed value.
static void initialize_movement_stats() {
    static bool loaded = false;
    if (loaded) {
        return;
    }
    loaded = true;

    // Stats store play time in minutes; playback_ticks counts state-machine ticks.
    playback_ticks  = (uint32_t)(stat_get_play_time_min() * 60.0f * FREQUENCY_STATE_MACHINE_TICK);
    playback_travel = stat_get_tape_played_ft();
}

void movement_init() {
    initialize_movement_stats();
    integrator = 0.0f;
    ticks = -1;
    state = MV_IDLE;
    target.active = false;
    future_target.active = false;
    commanded_target.active = false;
    movement_set_target_idle();
    movement_controllers_init();
}

void movement_set_fault_disarm(bool disarmed) {
    fault_disarmed = disarmed;
}

void movement_set_auto_play(bool enabled) {
    auto_play = enabled;
    // Debug: PIN_DEBUG2 LED mirrors the auto-play flag (on = enabled).
    if (enabled) {
        gpio_set_pin(PIN_DEBUG2);
    } else {
        gpio_clear_pin(PIN_DEBUG2);
    }
}

void movement_tick() {
    gpio_set_pin(PIN_DEBUG1);
    ticks += 1;


    // Step 0: Update data
    prev_tape_position = data_collector_get_tape_position();
    data_collector_update();

    // Safety gate: while fault-disarmed, skip the motion pipeline and the pinch
    // roller. The motors are disabled at the driver, but we still command zero
    // torque every tick -- that keeps polling each motor's fault byte (the reply)
    // without driving, so a cleared fault is observed and CMD_PLAY can re-arm.
    // Faults are kept live here; only the disarm itself latches. Telemetry
    // (data_collector) above also stays live.
    if (fault_disarmed) {
        solenoid_pinch_disengage();
        motors_set_reel_torques(0.0f, 0.0f);
        gpio_clear_pin(PIN_DEBUG1);
        return;
    }

    // Step 0.5: Odometer stats
    if (state == MV_PLAYBACK) {
        playback_ticks++;
        playback_travel += (data_collector_get_tape_position() - prev_tape_position);
    }

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
        data_collector_get_tape_position(),
        &command);

    // Step 4: Transition states if they are ready to transition
    if (status == TRANSITION_READY) {
        if (state == MV_IDLE) {
            // Leaving idle: enter the pipeline at the point the target needs.
            if (target.active && target.kind != TARGET_IDLE) {
                set_state(target.kind == TARGET_PLAYBACK ? MV_PLAYBACK : START_TENSION);
            }
        } else {
            MovementState next = NEXT_ON_READY[state];
            // Auto-play: a finished SEEK rolls into playback rather than stopping.
            // Only when no explicit command is already queued (future_target wins).
            if (next == MV_IDLE && auto_play
                && target.kind == TARGET_SEEK && !future_target.active) {
                target.kind = TARGET_PLAYBACK;   // PLAYBACK carries no union params
                target.active = true;
                set_state(MV_PLAYBACK);
            } else {
                set_state(next);
                if (next == MV_IDLE) {
                    target.active = false;  // run finished; release the target slot
                }
            }
        }
    }

    // Step 5: Apply command to motors
    MotorCommand motor_command = get_motor_commands(command);

    motors_set_reel_torques(motor_command.u_takeup, motor_command.u_supply);

    //float supply_speed = bldc_set_torque_float(&BLDC_CONF_SUPPLY, motor_command.u_supply);
    //float takeup_speed = bldc_set_torque_float(&BLDC_CONF_TAKEUP, motor_command.u_takeup);

    float supply_speed = 0.0f;
    float takeup_speed = 0.0f;

    //uart_println_float(motor_command.u_takeup);
    
    data_collector_set_reel_speeds(takeup_speed, supply_speed);
    gpio_clear_pin(PIN_DEBUG1);
}

// Abandon a SEEK target's position goal, turning it into a plain WIND in the
// same direction/speed so DECELERATE uses the normal (non-mem) stop path.
// No-op for non-SEEK targets.
static void abandon_seek_goal() {
    if (target.kind != TARGET_SEEK) return;
    MovementDirection dir = target.u.seek.dir;
    float speed = target.u.seek.speed;
    target.kind = TARGET_WIND;
    target.u.wind.dir = dir;
    target.u.wind.speed = speed;
}

static void state_transition_commanded_target() {
    CommandTransition tr = CMD_TRANSITION[state];

    if (tr.set_next) {
        set_state(tr.next);
    }
    if (tr.abandon_seek) {
        abandon_seek_goal();
    }
    if (tr.load_now) {
        transfer_commanded_target_current();
    } else {
        transfer_commanded_target_future();
    }
}

void movement_set_target_ff(float tape_speed) {
    if (tape_speed < 0.0f) return;
    MovementTarget t = (MovementTarget) {
        .kind = TARGET_WIND,
        .active = true,
        .u.wind = { .dir = FORWARD, .speed = tape_speed },
    };
    set_commanded_target(t);
}

void movement_set_target_rew(float tape_speed) {
    if (tape_speed < 0.0f) return;
    MovementTarget t = (MovementTarget) {
        .kind = TARGET_WIND,
        .active = true,
        .u.wind = { .dir = REVERSE, .speed = tape_speed },
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
        .kind = TARGET_SEEK,
        .active = true,
        .u.seek = { .dir = dir, .speed = tape_speed, .position = tape_position },
    };
    set_commanded_target(t);
}

void movement_set_target_playback() {
    MovementTarget t = (MovementTarget) {
        .kind = TARGET_PLAYBACK,
        .active = true,
    };
    set_commanded_target(t);
}

void movement_set_target_idle() {
    MovementTarget t = (MovementTarget) {
        .kind = TARGET_IDLE,
        .active = true,
    };
    set_commanded_target(t);
}

static void set_commanded_target(MovementTarget t) {
    uint32_t primask = critical_enter();
    commanded_target = t;
    commanded_target.active = true;
    critical_exit(primask);
}

static void transfer_commanded_target_current() {
    uint32_t primask = critical_enter();
    target = commanded_target;
    commanded_target.active = false;
    target.active = true;
    future_target.active = false;
    critical_exit(primask);
}

static void transfer_commanded_target_future() {
    uint32_t primask = critical_enter();
    if (commanded_target.active) {
        future_target = commanded_target;
        commanded_target.active = false;
    }
    critical_exit(primask);
}

static void transfer_future_target_current() {
    uint32_t primask = critical_enter();
    target = future_target;
    future_target.active = false;
    critical_exit(primask);
}

static float get_primary_tension() {
    if (target_direction(target) == FORWARD) {
        return data_collector_get_takeup_tension();
    }
    return data_collector_get_supply_tension();
}

static float get_secondary_tension() {
    if (target_direction(target) == FORWARD) {
        return data_collector_get_supply_tension();
    }
    return data_collector_get_takeup_tension();
}

static float get_tape_speed() {
    float tape_vel = data_collector_get_tape_speed();

    if (target_direction(target) == FORWARD) {
        return tape_vel;
    }

    return -tape_vel;
}

static MotorCommand get_motor_commands(MovementCommand movement_command) {
    MotorCommand command;
    if (target_direction(target) == FORWARD) {
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
    ticks = -1;
    if (state == MV_PLAYBACK) {
        motors_capstan_enable();
        solenoid_pinch_engage();
    } else {
        motors_capstan_disable();
        solenoid_pinch_disengage();
    }
}

static char* state_name(MovementState s) {
    switch (s) {
        case MV_IDLE:       return "MV_IDLE";
        case MV_PLAYBACK:   return "MV_PLAYBACK";
        case START_TENSION: return "START_TENSION";
        case ACCELERATE:    return "ACCELERATE";
        case CLOSED_LOOP:   return "CLOSED_LOOP";
        case DECELERATE:    return "DECELERATE";
        case STOP_TENSION:  return "STOP_TENSION";
    }
    return "UNKNOWN";
}

static char* dir_name(MovementDirection d) {
    return (d == FORWARD) ? "FWD" : "REV";
}

static char* kind_name(TargetKind k) {
    switch (k) {
        case TARGET_IDLE:     return "IDLE";
        case TARGET_PLAYBACK: return "PLAYBACK";
        case TARGET_WIND:     return "WIND";
        case TARGET_SEEK:     return "SEEK";
    }
    return "UNKNOWN";
}

char* movement_state_name(void) {
    return state_name(state);
}

void movement_debug_print_on_change(void) {
    // Consistent snapshot of the state machine and its current target.
    uint32_t primask = critical_enter();
    MovementState s = state;
    MovementTarget t = target;
    critical_exit(primask);

    // Canonical, kind-safe view of the target payload (0 where not applicable).
    MovementDirection dir = target_direction(t);
    float speed = (t.kind == TARGET_WIND) ? t.u.wind.speed
                : (t.kind == TARGET_SEEK) ? t.u.seek.speed : 0.0f;
    float pos = (t.kind == TARGET_SEEK) ? t.u.seek.position : 0.0f;

    static bool initialized = false;
    static MovementState last_state;
    static bool last_active;
    static TargetKind last_kind;
    static MovementDirection last_dir;
    static float last_speed;
    static float last_pos;

    bool changed = !initialized
        || s != last_state
        || t.active != last_active
        || t.kind != last_kind
        || dir != last_dir
        || speed != last_speed
        || pos != last_pos;

    if (!changed) return;

    initialized = true;
    last_state = s;
    last_active = t.active;
    last_kind = t.kind;
    last_dir = dir;
    last_speed = speed;
    last_pos = pos;

    uart_print("[MOV] ");
    uart_print(state_name(s));
    uart_print(" | tgt=");
    uart_print(kind_name(t.kind));
    if (!t.active) {
        uart_print("(inactive)");
    }
    if (t.kind == TARGET_WIND || t.kind == TARGET_SEEK) {
        uart_print(" dir=");
        uart_print(dir_name(dir));
        uart_print(" spd=");
        uart_print_float(speed);
    }
    if (t.kind == TARGET_SEEK) {
        uart_print(" pos=");
        uart_print_float(pos);
    }
    uart_println("");
}

void movement_get_ui_state(UiState* out_state, bool* out_transient) {
    uint32_t primask = critical_enter();
    MovementState s = state;
    MovementTarget t = target;
    critical_exit(primask);

    UiState ui;
    if (s == MV_IDLE) {
        ui = UI_STATE_IDLE;
    } else if (s == MV_PLAYBACK) {
        ui = UI_STATE_PLAY;
    } else {
        switch (t.kind) {
            case TARGET_IDLE:     ui = UI_STATE_IDLE; break;
            case TARGET_PLAYBACK: ui = UI_STATE_PLAY; break;
            case TARGET_SEEK:     ui = UI_STATE_SEEKING; break;
            case TARGET_WIND:
                ui = (t.u.wind.dir == FORWARD) ? UI_STATE_FAST_FORWARD : UI_STATE_REWIND;
                break;
        }
    }

    *out_state = ui;
    *out_transient = (s == START_TENSION
                  || s == ACCELERATE
                  || s == DECELERATE
                  || s == STOP_TENSION);
}

