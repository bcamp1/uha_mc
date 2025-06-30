/*
 * motor_unit.c
 *
 * Created: 3/28/2025 10:48:53 PM
 *  Author: brans
 */ 

#include "motor_unit.h"
#include "../foc/foc_math_fpu.h"
#include "../foc/foc.h"
#include "../periphs/uart.h"
#include "../periphs/gpio.h"
#include "board.h"

const MotorUnitConfig MOTOR_UNIT_A = {
	.driver = &UHA_MTR_DRVR_CONF_A,
	.encoder = &MOTOR_ENCODER_A,
};

const MotorUnitConfig MOTOR_UNIT_B = {
	.driver = &UHA_MTR_DRVR_CONF_B,
	.encoder = &MOTOR_ENCODER_B,
};

void motor_unit_init(const MotorUnitConfig* config) {
	// Initialize driver
	uha_motor_driver_init(config->driver);
	
	// Initialize Encoder
	motor_encoder_init(config->encoder);
}

#define TORQUE_LIMIT (0.4f)

void motor_unit_set_torque(const MotorUnitConfig* config, float torque) {
	// Get encoder position and target position (90 degree lead)
	
	//float target_pos = encoder_pos + (0.5f * PI);
	//if (torque < 0) {
		//target_pos = encoder_pos - (0.5f * PI);
	//}
    
	float encoder_pos = motor_encoder_get_pole_position(config->encoder);

    gpio_set_pin(DEBUG_PIN);
	 // Scale down torque + saturate
	 if (torque > 1.0f) torque = 1.0f;
	 if (torque < -1.0f) torque = -1.0f;
	 torque *= TORQUE_LIMIT;
	 
	 // Get PWM values
	 float a = 0;
	 float b = 0;
	 float c = 0;
	 float d = 0;
	 float q = torque;

	 foc_get_duties(encoder_pos, d, q, &a, &b, &c);
	 
	 // Convert to integers
	 uint8_t a_int = (int) (255.0f * a);
	 uint8_t b_int = (int) (255.0f * b);
	 uint8_t c_int = (int) (255.0f * c);
	 
	 // Write them to motor driver
	 uha_motor_driver_set_pwm(config->driver, a_int, b_int, c_int);
     gpio_clear_pin(DEBUG_PIN);
}

void motor_unit_energize_coils(const MotorUnitConfig* config, float a, float b, float c) {
	// Convert to integers
	uint8_t a_int = (int) (255.0f * a);
	uint8_t b_int = (int) (255.0f * b);
	uint8_t c_int = (int) (255.0f * c);
	
	// Write them to motor driver
	uha_motor_driver_set_pwm(config->driver, a_int, b_int, c_int);
}
