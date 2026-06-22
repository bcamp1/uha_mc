/*
 * stat_tracker.h
 *
 * Persistent usage statistics, stored in SmartEEPROM (see seeprom.h). Layers
 * float getters/setters over fixed byte offsets so callers never touch raw
 * EEPROM addresses. Currently tracks total play time and total tape played; add
 * new stats by claiming the next free offset in stat_tracker.c.
 *
 * Requires SmartEEPROM to be up (seeprom_init_or_provision()) before use.
 */

#pragma once

// Baseline any still-uninitialized stat to 0.0 (a freshly provisioned EEPROM
// reads back as NaN). Optional -- the getters already treat uninitialized as 0 --
// but calling it once after seeprom init writes clean zeros to flash. Safe to
// call every boot; it only writes stats that are still in the erased state.
void stat_tracker_init(void);

// Total play time, in minutes.
float stat_get_play_time_min(void);
void  stat_set_play_time_min(float minutes);

// Total tape played, in feet.
float stat_get_tape_played_ft(void);
void  stat_set_tape_played_ft(float feet);

// Set up a low-priority timer that periodically (~5 s) snapshots the live
// playback figures from movement and writes them to the EEPROM, so the persisted
// stats stay current without the caller polling. Call once after seeprom is up.
void schedule_stat_flusher(void);
