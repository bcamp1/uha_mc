#include <stdbool.h>

typedef enum {
    STOPPED,
    FF,
    REW,
    PLAYBACK,
    FF_TO_STOP,
    REW_TO_STOP,
    PLAYBACK_TO_STOP,
} State;

typedef enum {
    PLAY_ACTION,
    STOP_ACTION,
    FF_ACTION,
    REW_ACTION
} StateAction;

void state_machine_init();
void state_machine_take_action(StateAction a);
void state_machine_tick();

