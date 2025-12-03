#pragma once
#include "filter.h"

extern Filter controller_ff_takeup;
extern Filter controller_ff_supply;

extern Filter controller_rew_takeup;
extern Filter controller_rew_supply;

extern Filter controller_playback_takeup;
extern Filter controller_playback_supply;

extern Filter controller_idle_takeup;
extern Filter controller_idle_supply;

extern Filter controller_tape_speed_ff;
extern Filter controller_tape_speed_rew;

void controllers_init_all(float T);

