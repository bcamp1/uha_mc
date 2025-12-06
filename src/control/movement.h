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

void movement_init();
void movement_tick();
void movement_set_target_ff(float tape_speed);
void movement_set_target_playback();
void movement_set_target_idle();
void movement_set_target_rew(float tape_speed);
void movement_set_target_mem(float tape_position, float tape_speed);

