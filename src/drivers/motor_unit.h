/*
 * motor_unit.h
 *
 * Created: 3/28/2025 10:49:03 PM
 *  Author: brans
 */ 


#ifndef MOTOR_UNIT_H_
#define MOTOR_UNIT_H_

#include "motor_encoder.h"
#include "uha_motor_driver.h"

typedef struct {
	const UHAMotorDriverConfig* driver;
	const MotorEncoderConfig* encoder;
	
} MotorUnitConfig;

extern const MotorUnitConfig MOTOR_UNIT_A;
extern const MotorUnitConfig MOTOR_UNIT_B;

void motor_unit_init(const MotorUnitConfig* config);
void motor_unit_set_torque(const MotorUnitConfig* config, float torque);
void motor_unit_energize_coils(const MotorUnitConfig* config, float a, float b, float c);


#endif /* MOTOR_UNIT_H_ */
