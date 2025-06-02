
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

static void controller_func(SensorState e_x, SensorState e_v, SensorState e_a, SensorState e_i, float* torque1, float* torque2) {
    float k1[6] = {
        0.0f, // Theta 1
        0.0f, // Theta 2
        0.0f, // Tape Position
        -0.03f, // Tape Speed
        -0.0f, // Tension Arm 1
        0.0f, // Tension Arm 2
    };

    float k2[6] = {
        0.0f, // Theta 1
        0.0f, // Theta 2
        0.0f, // Tape Position
        -0.01f, // Tape Speed
        0.3f, // Tension Arm 1
        0.0f, // Tension Arm 2
    };

    float k1_i[6] = {
        0.0f, // Theta 1
        0.0f, // Theta 2
        0.0f, // Tape Position
        -0.01f, // Tape Speed
        0.0f, // Tension Arm 1
        0.0f, // Tension Arm 2
    };

    float k2_i[6] = {
        0.0f, // Theta 1
        0.0f, // Theta 2
        0.0f, // Tape Position
        -0.00f, // Tape Speed
        0.0f, // Tension Arm 1
        0.0f, // Tension Arm 2
    };

    float k1_d[6] = {
        0.0f, // Theta 1
        0.0f, // Theta 2
        0.0f, // Tape Position
        -0.03f, // Tape Speed
        0.0f, // Tension Arm 1
        0.0f, // Tension Arm 2
    };

    float k2_d[6] = {
        0.0f, // Theta 1
        0.0f, // Theta 2
        0.0f, // Tape Position
        -0.03f, // Tape Speed
        0.05f, // Tension Arm 1
        0.0f, // Tension Arm 2
    };

    *torque1 = sensor_state_dot(e_x, k1) 
        + sensor_state_dot(e_i, k1_i)
        + sensor_state_dot(e_v, k1_d);

    *torque2 = sensor_state_dot(e_x, k2)
        + sensor_state_dot(e_i, k2_i)
        + sensor_state_dot(e_v, k2_d);
}


static ControllerReference r = {
    .theta1_dot = 0.0f,
    .theta2_dot = 0.0f,
    .tape_speed = 15.0f,
    .tension1 = 0.8f,
    .tension2 = 0.0f,
};

ControllerConfig controller_tests_config = {
	.controller = controller_func,
    .reference = &r,
};

void controller_tests_run(ControllerConfig *config, bool send_logs, bool uart_toggle, bool start_on) { 
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
    char input_command = 0;
	while (1) {
        //controller_print_state();
        if (send_logs && state_recorder_should_transmit()) {
            input_command = uart_get();
            state_recorder_transmit();
        } else if (!send_logs) {
            input_command = uart_get();
        } else {
            input_command = 0;
        }

        if (uart_toggle) {
            if (input_command == 'e') {
                motors_enabled = true;
                controller_enable_motors();
                gpio_set_pin(LED);
                if (!send_logs) {
                    uart_println("Enabling Motors. Type 'e' to disable.");
                }
            } else if (input_command == 'p') {
                motors_enabled = false;
                controller_disable_motors();
                gpio_clear_pin(LED);
                if (!send_logs) {
                    uart_println("Disabling Motors. Type 'e' to enable.");
                }
            }
        }
        
    }
}
