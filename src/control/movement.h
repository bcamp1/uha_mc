#pragma once
#include <stdbool.h>

typedef enum {FORWARD, REVERSE} MovementDirection;

typedef struct {
    float u_takeup;
    float u_supply;
} MotorCommand;

typedef struct {
    float u_primary;
    float u_secondary;
} MovementCommand;

// What the deck has been asked to do. `kind` selects which union arm (if any)
// carries the parameters: IDLE and PLAYBACK carry none, WIND/SEEK carry the
// fields valid for that mode. `active` is the lifecycle flag (does this slot
// hold a real request) and is orthogonal to `kind`.
typedef enum {
    TARGET_IDLE,
    TARGET_PLAYBACK,
    TARGET_WIND,   // fast-forward / rewind at a speed
    TARGET_SEEK,   // wind to a tape position ("mem")
} TargetKind;

typedef struct {
    TargetKind kind;
    bool active;
    union {
        struct {
            MovementDirection dir;
            float speed;
        } wind;
        struct {
            MovementDirection dir;
            float speed;
            float position;
        } seek;
    } u;
} MovementTarget;

// Direction of travel for any target. IDLE and PLAYBACK always run FORWARD.
static inline MovementDirection target_direction(MovementTarget t) {
    switch (t.kind) {
        case TARGET_WIND: return t.u.wind.dir;
        case TARGET_SEEK: return t.u.seek.dir;
        default:          return FORWARD;
    }
}

typedef enum {
    MV_IDLE,
    MV_PLAYBACK,
    START_TENSION,
    ACCELERATE,
    CLOSED_LOOP,
    DECELERATE,
    STOP_TENSION,
} MovementState;

// User-facing rollup of (MovementState, MovementTarget). Reflects what the
// deck is trying to do, not which controller phase is running. Pair with
// `transient` from movement_get_ui_state() to know if the deck is mid-ramp.
typedef enum {
    UI_STATE_IDLE         = 0,
    UI_STATE_PLAY         = 1,
    UI_STATE_FAST_FORWARD = 2,
    UI_STATE_REWIND       = 3,
    UI_STATE_SEEKING      = 4,
} UiState;

void movement_init();
void movement_tick();
void movement_set_target_ff(float tape_speed);
void movement_set_target_playback();
void movement_set_target_idle();
void movement_set_target_rew(float tape_speed);
void movement_set_target_mem(float tape_position, float tape_speed);
void movement_get_ui_state(UiState* out_state, bool* out_transient);

// Name of the current state-machine state (e.g. "ACCELERATE"), for debug/UART.
char* movement_state_name(void);

// Print "[MOV] <state> | tgt=..." over UART, but only when the state or target
// has changed since the last call. Safe to call every iteration of a poll loop.
void movement_debug_print_on_change(void);

