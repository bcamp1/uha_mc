#include "controller.h"
#include <stdlib.h>
#include <math.h>

#include "../drivers/motor_unit.h"
#include "../drivers/uha_motor_driver.h"
#include "../drivers/motor_encoder.h"
#include "../drivers/tension_arm.h"
#include "../drivers/inc_encoder.h"
#include "../periphs/gpio.h"
#include "../periphs/uart.h"

#define DEBUG_PIN PIN_PA14

static ControllerConfig* config = NULL;
static State x_k;
static float theta1_prev;
static float theta2_prev;
static float tension1_prev;
static float tension2_prev;

static float get_delta_angle(float prev, float new) {
    static const float twopi = 2.0f * 3.14159f;
    float a = new - prev;
    float b = (new + twopi) - prev;
    float c = new - (prev + twopi);

    float abs_a = fabs(a);
    float abs_b = fabs(b);
    float abs_c = fabs(c);

    if (abs_a <= abs_b && abs_a <= abs_c) {
        return a;
    } else if (abs_b <= abs_a && abs_b <= abs_c) {
        return b;
    } else {
        return c;
    }
}

State controller_get_state() {
    return x_k;
}

void controller_send_state_uart() {
    uart_send_float(x_k.theta1); 
    uart_send_float(x_k.theta2); 
    uart_send_float(x_k.theta1_dot); 
    uart_send_float(x_k.theta2_dot); 
    uart_send_float(x_k.tension1); 
    uart_send_float(x_k.tension2); 
    uart_send_float(x_k.tension1_dot); 
    uart_send_float(x_k.tension2_dot); 
    uart_send_float(x_k.tape_speed); 
    //uart_send_float(INFINITY);
    //uart_println_float(x_k.tension1);
    //uart_println_float(x_k.tension2);
}

void controller_init_all_hardware() {
	// Init hardware
    motor_unit_init(&MOTOR_UNIT_A);
	motor_unit_init(&MOTOR_UNIT_B);
    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);
    
    // Init state
    x_k.theta1 = 0;
    x_k.theta2 = 0;
    x_k.theta1_dot = 0;
    x_k.theta2_dot = 0;
    x_k.tape_speed = 0;
    x_k.tension1 = 0;
    x_k.tension1_dot = 0;
    x_k.tension2 = 0;
    x_k.tension2_dot = 0;
}

void controller_set_config(ControllerConfig* c) {
    config = c;
}

void controller_run_iteration() {
    theta1_prev = x_k.theta1;
    theta2_prev = x_k.theta2;
    tension1_prev = x_k.tension1;
    tension2_prev = x_k.tension2;

    // Get new absolute positions
    x_k.theta1 = motor_encoder_get_position(&MOTOR_ENCODER_A);
    x_k.theta2 = motor_encoder_get_position(&MOTOR_ENCODER_B);
    x_k.tension1 = tension_arm_get_position(&TENSION_ARM_A);
    x_k.tension2 = tension_arm_get_position(&TENSION_ARM_B);
    uart_send_float(INFINITY);
    //uart_println("");
    //uart_print("Hi! ");
    //uart_println_float(tension_arm_get_position(&TENSION_ARM_B));
    //uart_println_float(tension_arm_get_position(&TENSION_ARM_B));
    //uart_println_float(tension_arm_get_position(&TENSION_ARM_B));
    
    // Use backwards euler to get absolute velocities
    float T = config->sample_period;
    x_k.theta1_dot = get_delta_angle(theta1_prev, x_k.theta1) / T;
    x_k.theta2_dot = get_delta_angle(theta2_prev, x_k.theta2) / T;
    x_k.tension1_dot = (x_k.tension1 - tension1_prev) / T;
    x_k.tension2_dot = (x_k.tension2 - tension2_prev) / T;

    // Get inc encoder values
    x_k.tape_speed = inc_encoder_get_vel();

    // Get torques
    //float torque1 = config->controller1(x_k);
    //float torque2 = config->controller2(x_k);
    float torque1 = 0;
    float torque2 = 0;

    // Send torques to motor
	gpio_set_pin(DEBUG_PIN);
    motor_unit_set_torque(&MOTOR_UNIT_A, torque1);
    motor_unit_set_torque(&MOTOR_UNIT_B, torque2);
	gpio_clear_pin(DEBUG_PIN);
}

void controller_run_loop() {
    while (1) {
        controller_run_iteration();
    }
}
