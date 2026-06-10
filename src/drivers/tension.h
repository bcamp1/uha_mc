/*
 * ems22a.h
 *
 * Created: 3/20/2025 12:35:38 AM
 *  Author: brans
 */ 

#pragma once
#include <stdbool.h>

void tension_init();
void tension_init_takeup_only();
void tension_init_supply_only();

// Information
float tension_get_takeup();
float tension_get_takeup_raw();
float tension_get_supply();
float tension_get_supply_raw();

// Arm error state, judged from the last cached raw frame (no SPI access). True
// when the EMS22A frame is stuck (0x0000/0xFFFF) or fails its parity check.
bool tension_takeup_error();
bool tension_supply_error();

// Calibration
void tension_calibrate_takeup_bottom();
void tension_calibrate_takeup_top();
void tension_calibrate_supply_bottom();
void tension_calibrate_supply_top();

