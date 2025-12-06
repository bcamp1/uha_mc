#pragma once
#include "movement.h"
#include <stdint.h>

typedef enum {TRANSITION_READY, TRANSITION_NOT_READY} TransitionStatus;

void movement_controllers_init();
TransitionStatus movement_controllers_tick(MovementState state, MovementTarget target, uint32_t ticks, float t_p, float t_s, float tape_speed, MovementCommand* command);

