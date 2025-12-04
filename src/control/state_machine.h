#pragma once
#include <stdbool.h>

typedef enum {
    STOPPED,
    FF,
    REW,
    PLAYBACK,
    FF_TO_IDLE,
    REW_TO_IDLE,
    PLAYBACK_TO_IDLE,
    IDLE,
    GOTO_MEM,
} State;

typedef enum {
    PLAY_ACTION,
    STOP_ACTION,
    FF_ACTION,
    REW_ACTION,
    MEM_ACTION,
    NO_ACTION,
} StateAction;

void state_machine_init();
void state_machine_take_action(StateAction a);
void state_machine_tick();
void state_machine_goto_position(float position);

