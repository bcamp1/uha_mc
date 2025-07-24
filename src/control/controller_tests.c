#include "controller.h"
#include "../drivers/uha_motor_driver.h"
#include "../drivers/delay.h"
#include "../drivers/board.h"
#include "../periphs/uart.h"
#include "../drivers/stopwatch.h"
#include "../drivers/spi_collector.h"
#include "state_recorder.h"
#include "../periphs/gpio.h"
#include "control_state.h"
#include <stdbool.h>

static void controller_func(ControlState error, float* torque1, float* torque2) {
    ControlState K_takeup = {
        .takeup_reel_speed = 0.0f, // 0.5
        .takeup_reel_acceleration = 0.0f, // 0.1
        .takeup_tension = 0.8f,
        .takeup_tension_speed = 0.0f,
        .supply_reel_speed = 0.0f,
        .supply_reel_acceleration = 0.0f,
        .supply_tension = 0.0f,
        .supply_tension_speed = 0.0f,
        .tape_position = 0.0f,
        .tape_speed = -0.00f,
        .tape_acceleration = -0.00f,
    };


    ControlState K_supply = {
        .takeup_reel_speed = 0.0f,
        .takeup_reel_acceleration = 0.0f,
        .takeup_tension = 0.0f,
        .takeup_tension_speed = 0.0f,
        .supply_reel_speed = 0.0f,
        .supply_reel_acceleration = 0.0f,
        .supply_tension = -0.8f,
        .supply_tension_speed = -0.0f,
        .tape_position = 0.0f,
        .tape_speed = 0.0f,
        .tape_acceleration = 0.0f,
    };
    
    *torque1 = control_state_dot(&K_takeup, &error);
    *torque2 = control_state_dot(&K_supply, &error);
}

static ControlState r = {
    .takeup_reel_speed = 0.0f,
    .takeup_reel_acceleration = 0.0f,
    .takeup_tension = 0.5f,
    .takeup_tension_speed = 0.0f,

    .supply_reel_speed = 0.0f,
    .supply_reel_acceleration = 0.0f,
    .supply_tension = 0.7f,
    .supply_tension_speed = 0.0f,

    .tape_position = 0.0f,
    .tape_speed = 15.0f,
    .tape_acceleration = 0.0f,
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
    //controller_start_process();
    spi_collector_enable_service();

    char input_command = 0;
	while (1) {
        //gpio_set_pin(DEBUG_PIN);
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
                gpio_set_pin(LED_PIN);
                if (!send_logs) {
                    uart_println("Enabling Motors. Type 'e' to disable.");
                }
            } else if (input_command == 'p') {
                motors_enabled = false;
                controller_disable_motors();
                gpio_clear_pin(LED_PIN);
                if (!send_logs) {
                    uart_println("Disabling Motors. Type 'e' to enable.");
                }
            }
        }
        
    }
}

static void controller_func_rewind(ControlState error, float* torque1, float* torque2) {
    ControlState K_takeup = {
        .takeup_reel_speed = 0.0f, // 0.5
        .takeup_reel_acceleration = 0.0f, // 0.1
        .takeup_tension = -0.5f,
        .takeup_tension_speed = -0.5f,
        .supply_reel_speed = 0.0f,
        .supply_reel_acceleration = 0.0f,
        .supply_tension = 0.0f,
        .supply_tension_speed = 0.0f,
        .tape_position = 0.0f,
        .tape_speed = -0.00f,
        .tape_acceleration = -0.0f,
    };

    ControlState K_supply = {
        .takeup_reel_speed = 0.0f,
        .takeup_reel_acceleration = 0.0f,
        .takeup_tension = 0.0f,
        .takeup_tension_speed = 0.0f,
        .supply_reel_speed = 0.1f,
        .supply_reel_acceleration = 0.00f,
        .supply_tension = 0.0f,
        .supply_tension_speed = 0.0f,
        .tape_position = 0.0f,
        .tape_speed = 0.0f,
        .tape_acceleration = 0.0f,
    };
    
    *torque1 = control_state_dot(&K_takeup, &error);
    *torque2 = control_state_dot(&K_supply, &error);
}

static ControlState r_rewind = {
    .takeup_reel_speed = 0.0f,
    .takeup_reel_acceleration = 0.0f,
    .takeup_tension = 0.5f,
    .takeup_tension_speed = 0.0f,

    .supply_reel_speed = 20.0f,
    .supply_reel_acceleration = 0.0f,
    .supply_tension = 0.5f,
    .supply_tension_speed = 0.0f,

    .tape_position = 0.0f,
    .tape_speed = -15.0f,
    .tape_acceleration = 0.0f,
};

ControllerConfig controller_tests_config_rewind = {
	.controller = controller_func_rewind,
    .reference = &r_rewind,
};
