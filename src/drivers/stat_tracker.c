/*
 * stat_tracker.c
 *
 * Persistent usage statistics on top of the SmartEEPROM driver. See stat_tracker.h.
 */

#include "stat_tracker.h"
#include "seeprom.h"
#include "../periphs/timer.h"
#include "../control/movement.h"
#include "../sched.h"
#include <stdbool.h>
#include <stdint.h>

// --- SmartEEPROM byte-offset map ---------------------------------------------
// Unique offsets within SEEPROM_SIZE (100). Byte 0 is the boot counter owned by
// main.c, so stats live from offset 4 upward, each a 4-byte float. To add a stat
// later, claim the next free 4-byte slot and bump the "next free" note below.
//
//   0      boot counter        (uint8,  main.c)
//   4..7   total play time     (float minutes)
//   8..11  total tape played   (float feet)
//
#define STAT_ADDR_PLAY_TIME_MIN   4
#define STAT_ADDR_TAPE_PLAYED_FT  8
// next free offset: 12

// A freshly provisioned cell reads back 0xFFFFFFFF, which decodes to a float NaN.
// (v != v) is true only for NaN, so we use it to detect the never-written state.
static bool stat_uninitialized(uint16_t addr) {
	float v = seeprom_read_float(addr);
	return v != v;
}

// Read a stat, reporting an uninitialized (erased) slot as 0 so totals start clean.
static float read_stat(uint16_t addr) {
	float v = seeprom_read_float(addr);
	return (v != v) ? 0.0f : v;
}

void stat_tracker_init(void) {
	if (stat_uninitialized(STAT_ADDR_PLAY_TIME_MIN)) {
		seeprom_write_float(STAT_ADDR_PLAY_TIME_MIN, 0.0f);
	}
	if (stat_uninitialized(STAT_ADDR_TAPE_PLAYED_FT)) {
		seeprom_write_float(STAT_ADDR_TAPE_PLAYED_FT, 0.0f);
	}
}

float stat_get_play_time_min(void) {
	return read_stat(STAT_ADDR_PLAY_TIME_MIN);
}

void stat_set_play_time_min(float minutes) {
	seeprom_write_float(STAT_ADDR_PLAY_TIME_MIN, minutes);
}

float stat_get_tape_played_ft(void) {
	return read_stat(STAT_ADDR_TAPE_PLAYED_FT);
}

void stat_set_tape_played_ft(float feet) {
	seeprom_write_float(STAT_ADDR_TAPE_PLAYED_FT, feet);
}

// --- Periodic flusher --------------------------------------------------------
// The scheduler slot ticks at FREQUENCY_STAT_FLUSH (10 Hz); divide it down so the
// EEPROM is actually written about every 5 s.
#define STAT_FLUSH_DIVIDER ((uint32_t)(FREQUENCY_STAT_FLUSH * 5.0f))   // 50 ticks

static volatile uint32_t flush_div = 0;

static void stat_flush_tick(void) {
	if (++flush_div < STAT_FLUSH_DIVIDER) {
		return;
	}
	flush_div = 0;

	// Snapshot the fresh figures from movement. Each getter is a single volatile
	// word read, so it is safe to sample from this timer ISR. movement reports
	// play time in seconds; our stat is minutes.
	float minutes = movement_get_playback_time() / 60.0f;
	float feet    = movement_get_playback_travel();

	// These are blocking flash writes, but acceptable here: SmartEEPROM lives in
	// the read-while-write flash section (code keeps executing during the commit),
	// this is the lowest-priority timer, and it only fires every ~5 s.
	stat_set_play_time_min(minutes);
	stat_set_tape_played_ft(feet);
}

void schedule_stat_flusher(void) {
	timer_schedule(ID_STAT_FLUSH, FREQUENCY_STAT_FLUSH, PRIO_STAT_FLUSH, stat_flush_tick);
}
