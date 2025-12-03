#include "controllers.h"

Filter controller_ff_takeup;
Filter controller_ff_supply;

Filter controller_rew_takeup;
Filter controller_rew_supply;

Filter controller_playback_takeup;
Filter controller_playback_supply;

Filter controller_idle_takeup;
Filter controller_idle_supply;

Filter controller_tape_speed_ff;
Filter controller_tape_speed_rew;

static void init_playback(float T);
static void init_ff(float T);
static void init_rew(float T);
static void init_idle(float T);
static void init_tape_speed(float T);

void controllers_init_all(float T) {
    init_playback(T);
    init_ff(T);
    init_rew(T);
    init_idle(T);
    init_tape_speed(T);
}

static void init_playback(float T) {
    const float takeup_P = 1.5f;
    const float takeup_D = 0.08f;
    filter_init_pd(&controller_playback_takeup, -takeup_P, -takeup_D, T);

    const float supply_P = 1.5f;
    const float supply_D = 0.08f;
    filter_init_pd(&controller_playback_supply, supply_P, supply_D, T);
}

static void init_ff(float T) {
    const float takeup_P = 1.0f;
    const float takeup_D = 0.00f;
    filter_init_pd(&controller_ff_takeup, -takeup_P, -takeup_D, T);

    const float supply_P = 1.5f;
    const float supply_D = 0.05f;
    filter_init_pd(&controller_ff_supply, supply_P, supply_D, T);
}

static void init_rew(float T) {
    const float takeup_P = 1.5f;
    const float takeup_D = 0.05f;
    filter_init_pd(&controller_rew_takeup, -takeup_P, -takeup_D, T);

    const float supply_P = 1.5f;
    const float supply_D = 0.05f;
    filter_init_pd(&controller_rew_supply, supply_P, supply_D, T);
}

static void init_idle(float T) {
    const float takeup_P = 1.5f;
    const float takeup_D = 0.05f;
    filter_init_pd(&controller_idle_takeup, -takeup_P, -takeup_D, T);

    const float supply_P = 1.5f;
    const float supply_D = 0.05f;
    filter_init_pd(&controller_idle_supply, supply_P, supply_D, T);
}

static void init_tape_speed(float T) {
    const float takeup_P = 1.5f;
    const float takeup_D = 0.00f;
    filter_init_pd(&controller_tape_speed_ff, -takeup_P, -takeup_D, T);

    const float supply_P = 1.5f;
    const float supply_D = 0.05f;
    filter_init_pd(&controller_tape_speed_rew, -supply_P, -supply_D, T);
}

