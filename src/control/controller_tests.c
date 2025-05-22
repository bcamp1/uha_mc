
#include "controller.h"
#include "../drivers/uha_motor_driver.h"
#include "../drivers/delay.h"
#include "../periphs/uart.h"
#include "../drivers/stopwatch.h"
#include "state_recorder.h"
#include <stdbool.h>
#define PI (3.14159f)

static void controller_demo(State x, float* torque1, float* torque2) {
	float theta1 = x.theta1;
	float theta2 = x.theta2;
	float k1 = 0.3f;
	float k2 = -0.5f;
	float delta1 = theta1 - PI;
	float delta2 = theta2 - PI;
	*torque1 = k1*delta2;
	*torque2 = k2*delta2;
}


static void controller_constant_torques(State x, float* torque1, float* torque2) {
	*torque1 = 0.4f;
	*torque2 = -0.4f;
}

static void controller_tensions(State x, float* torque1, float* torque2) {
	// References
	float t_target = 0.5f;
	float tape_speed_target = 3.0f;

	// Controller Constants
	float k_d = -0.8f;
	float k2_t1 = 0.5f;
	float k2_t2 = 0.8f;

	// Errors
	float t1_error = t_target - x.tension1;
	float t2_error = t_target - x.tension2;
	float tape_speed = -x.theta1_dot;
	float tape_speed_error = tape_speed_target - tape_speed;	

	// Compute controller output
	*torque1 = k_d*tape_speed_error; 
	*torque2 = k2_t1*t1_error + k2_t2*t2_error; 
}

ControllerConfig controller_config_demo = {
	.controller = controller_demo,
	.sample_period = 0.01f,
};

ControllerConfig controller_config_tensions = {
	.controller = controller_tensions,
	.sample_period = 0.01f,
};

ControllerConfig controller_config_constant = {
	.controller = controller_constant_torques,
	.sample_period = 0.01f,
};

void controller_tests_run(ControllerConfig *config, bool send_logs, bool uart_toggle, bool start_on) { 
    //uart_println("DEBUG");
	bool motors_enabled = true;
	controller_init_all_hardware();

	if (!start_on) {
		motors_enabled = false;
		controller_disable_motors();
	}

    if (send_logs) {
        state_recorder_schedule();
    }

	controller_set_config(config); 
    controller_start_process();
	while (1) {
        if (state_recorder_should_transmit()) {
            state_recorder_transmit();
        }
		
		if (uart_toggle) {
			if (uart_get() == 'e') {
				if (motors_enabled) {
					motors_enabled = false;
					controller_disable_motors();
					if (!send_logs) {
						uart_println("Disabling Motors. Type 'e' to enable.");
					}
				} else {
					motors_enabled = true;
					controller_enable_motors();
					if (!send_logs) {
						uart_println("Enabling Motors. Type 'e' to disable.");
					}
				}
			}
		}
    }
}
