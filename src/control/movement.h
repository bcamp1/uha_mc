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

typedef struct {
    MovementDirection direction;
    float tape_speed;
    float tape_position;
    bool goto_position;
    bool active;
    bool is_playback;
    bool is_idle;
} MovementTarget;

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

