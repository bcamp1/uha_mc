#pragma once

typedef enum {FORWARD, REVERSE} MovementDirection;

typedef struct {
    float u_takeup;
    float u_supply;
} ControlCommand;

typedef struct {
    float u_primary;
    float u_secondary;
} MovementCommand;

typedef enum {
    IDLE,
    STARTUP,
    CLOSED_LOOP,
    SLOWDOWN,
} MovementState;

