
#include "controller.h"
#include "../drivers/uha_motor_driver.h"
#include "../drivers/delay.h"
#include "../periphs/uart.h"
#include "../drivers/stopwatch.h"
#include "state_recorder.h"
#include "../periphs/gpio.h"
#include <stdbool.h>
#define PI (3.14159f)
#define DEBUG_PIN PIN_PA14
#define LED PIN_PA15

static float tape_speed_integrator = 0.0f;

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

static LinearControlLaw K = {
    .motor1_k = {
        0.0f, // Time
        0.0f, // Theta 1 
        0.0f, // Theta 2 
        0.0f, // Theta 1 dot 
        0.0f, // Theta 2 dot 
        0.0f, // Tape position 
        -0.03f, // Tape speed 
        -0.7f, // Tension 1 
        0.0f, // Tension 2 
        -0.0f, // Tension 1 dot 
        0.0f, // Tension 2 dot 
    },
    .motor2_k = {
        0.0f, // Time
        0.0f, // Theta 1 
        0.0f, // Theta 2 
        0.0f, // Theta 1 dot 
        0.0f, // Theta 2 dot 
        0.0f, // Tape position 
        -0.03f, // Tape speed 
        0.0f, // Tension 1 
        0.7f, // Tension 2 
        0.0f, // Tension 1 dot 
        0.0f, // Tension 2 dot 
    },
};

static State r = {
    .time = 0.0f,
    .theta1 = 0.0f,
    .theta2 = 0.0f,
    .theta1_dot = 0.0f,
    .theta2_dot = 0.0f,
    .tape_position = 0.0f,
    .tape_speed = 15.0f,
    .tension1 = 0.5f,
    .tension2 = 0.5f,
    .tension1_dot = 0.0f,
    .tension2_dot = 0.0f,
};

static void controller_linear(State x, float* torque1, float* torque2) {
    State e = controller_get_error(&r, &x);

    if (e.tension1_dot < 5.0f && e.tension1_dot > -5.0f) {
        e.tension1_dot = 0.0f;
    }
    if (e.tension2_dot < 5.0f && e.tension2_dot > -5.0f) {
        e.tension2_dot = 0.0f;
    }
    controller_linear_control_law(&K, &e, torque1, torque2);   
}

static void controller_pid(State x, float* torque1, float* torque2) {
    State e = controller_get_error(&r, &x);
    tape_speed_integrator += e.tape_speed;

    const float k_i = -0.0001f;

    if (e.tension1_dot < 5.0f && e.tension1_dot > -5.0f) {
        e.tension1_dot = 0.0f;
    }
    if (e.tension2_dot < 5.0f && e.tension2_dot > -5.0f) {
        e.tension2_dot = 0.0f;
    }
    controller_linear_control_law(&K, &e, torque1, torque2);   
    *torque1 += k_i*tape_speed_integrator;
    *torque2 += k_i*tape_speed_integrator;
}

ControllerConfig controller_config_demo = {
	.controller = controller_demo,
};

ControllerConfig controller_config_linear = {
	.controller = controller_linear,
};

ControllerConfig controller_config_pid = {
	.controller = controller_pid,
};

ControllerConfig controller_config_constant = {
	.controller = controller_constant_torques,
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
        if (!motors_enabled) {
            tape_speed_integrator = 0.0f;
        }
        if (state_recorder_should_transmit()) {
            char got = uart_get();
            state_recorder_transmit();
            if (uart_toggle) {
                if (got == 'e') {
                    if (motors_enabled) {
                        motors_enabled = false;
                        controller_disable_motors();
                        gpio_clear_pin(LED);
                        if (!send_logs) {
                            uart_println("Disabling Motors. Type 'e' to enable.");
                        }
                    } else {
                        motors_enabled = true;
                        controller_enable_motors();
                        gpio_set_pin(LED);
                        if (!send_logs) {
                            uart_println("Enabling Motors. Type 'e' to disable.");
                        }
                    }
                // For emergencies
                } else if (got == 'd') {
                    motors_enabled = false;
                    controller_disable_motors();
                    gpio_clear_pin(LED);
                    while (1) {}
                }
            }
        }
    }
}
