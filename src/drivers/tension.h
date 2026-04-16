/*
 * ems22a.h
 *
 * Created: 3/20/2025 12:35:38 AM
 *  Author: brans
 */ 

#pragma once

void tension_init();

// Information
float tension_get_takeup();
float tension_get_takeup_raw();
float tension_get_supply();
float tension_get_supply_raw();

// Calibration
void tension_calibrate_takeup_bottom();
void tension_calibrate_takeup_top();
void tension_calibrate_supply_bottom();
void tension_calibrate_supply_top();

