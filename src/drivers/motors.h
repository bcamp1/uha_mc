#pragma once
#include <stdint.h>
#include "motor_comms.h"

typedef enum {
    CAPSTAN_SPEED_15IPS,
    CAPSTAN_SPEED_30IPS,
    CAPSTAN_SPEED_7P5_IPS,
    CAPSTAN_SPEED_OTHER,
} CapstanSpeed;

void motors_init();
void motors_enable_all();
void motors_disable_all();
void motors_set_reel_torques(float takeup_torque, float supply_torque);

void motors_takeup_enable();
void motors_takeup_disable();
void motors_takeup_calibrate_encoder();
RXError motors_takeup_get_faults(uint8_t* faults);

void motors_supply_enable();
void motors_supply_disable();
void motors_supply_calibrate_encoder();
RXError motors_supply_get_faults(uint8_t* faults);

void motors_capstan_enable();
void motors_capstan_disable();
void motors_capstan_calibrate_encoder();
void motors_capstan_set_speed(CapstanSpeed speed);

