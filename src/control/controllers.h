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

void controllers_init_all(float T);

