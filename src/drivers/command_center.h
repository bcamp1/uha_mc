#pragma once

typedef enum {
    CMD_STOP,
    CMD_PLAY,
    CMD_FAST_FORWARD,
    CMD_REWIND,
    CMD_SPOOL,
    CMD_SET_ZERO,
    CMD_SET_CAPSTAN_SPEED,
    CMD_NONE, 
} CommandCenterSimpleAction;

void command_center_init();
CommandCenterSimpleAction command_center_get_action();

