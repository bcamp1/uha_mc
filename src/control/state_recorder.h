#ifndef STATE_RECORDER_H
#define STATE_RECORDER_H
#include <stdbool.h>

void state_recorder_schedule();
bool state_recorder_should_transmit();
void state_recorder_transmit();

#endif
