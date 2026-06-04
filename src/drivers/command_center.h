#pragma once

typedef enum {
    CMD_STOP,
    CMD_PLAY,
    CMD_FAST_FORWARD,
    CMD_REWIND,
    CMD_SPOOL,
    CMD_SET_ZERO,
    CMD_SET_CAPSTAN_SPEED,
    CMD_GOTO_LOC,
    CMD_NONE,
} CommandCenterSimpleAction;

void command_center_init();
CommandCenterSimpleAction command_center_get_action();

// Tape position (encoder units) requested by the most recent CMD_GOTO_LOC.
// Read after command_center_get_action() returns CMD_GOTO_LOC.
float command_center_get_goto_loc();

