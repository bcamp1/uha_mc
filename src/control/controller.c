#include "controller.h"
#include <stdlib.h>
#include <math.h>

#include "../drivers/motor_unit.h"
#include "../drivers/uha_motor_driver.h"
#include "../drivers/motor_encoder.h"
#include "../drivers/tension_arm.h"
#include "../drivers/stopwatch.h"
#include "../drivers/roller.h"
#include "../periphs/gpio.h"
#include "../periphs/uart.h"
#include "../periphs/spi.h"
#include "../periphs/timer.h"

#define DEBUG_PIN PIN_PA14
#define TWOPI (6.283185307179586f)
#define PI (3.14159f)

static volatile ControllerConfig* config = NULL;
static volatile State x_k;
static volatile float theta1_prev;
static volatile float theta2_prev;
static volatile float tension1_prev;
static volatile float tension2_prev;

void controller_print_tension_info() {
    uart_print("Tension Arm A: ");
    uart_print_float(tension_arm_get_position(&TENSION_ARM_A));
    uart_print(", Tension Arm B: ");
    uart_println_float(tension_arm_get_position(&TENSION_ARM_B));
}

void controller_print_encoder_info() {
	float encoder_pos_a = motor_encoder_get_pole_position(&MOTOR_ENCODER_A);
	float encoder_pos_b = motor_encoder_get_pole_position(&MOTOR_ENCODER_B);
	//float encoder_pos_b = motor_encoder_get_position(&MOTOR_ENCODER_B);
	uart_print("ENC A: ");
	uart_print_float(encoder_pos_a);
	uart_print(", ENC B: ");
	uart_println_float(encoder_pos_b);
}

static float get_delta_angle(float prev, float new) {
    float a = new - prev;
    float b = (new + TWOPI) - prev;
    float c = new - (prev + TWOPI);

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
    float data[10] = {
        x_k.theta1, 
        x_k.theta2, 
        x_k.theta1_dot, 
        x_k.theta2_dot, 
        x_k.tension1,
        x_k.tension2,
        x_k.tension1_dot,
        x_k.tension2_dot,
        x_k.tape_position,
        x_k.tape_speed};
    uart_send_float_arr(data, 10);
}

void controller_init_all_hardware() {
	// Init hardware
    motor_unit_init(&MOTOR_UNIT_A);
	motor_unit_init(&MOTOR_UNIT_B);
    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);
    roller_init(); 
    
    // Init state
    x_k.theta1 = 0;
    x_k.theta2 = 0;
    x_k.theta1_dot = 0;
    x_k.theta2_dot = 0;
    x_k.tape_speed = 0;
    x_k.tape_position = 0;
    x_k.tension1 = 0;
    x_k.tension1_dot = 0;
    x_k.tension2 = 0;
    x_k.tension2_dot = 0;
}

void controller_set_config(ControllerConfig* c) {
    config = c;
}

void controller_run_iteration() {
    gpio_set_pin(DEBUG_PIN);
    //stopwatch_start(1);
    // Update previous values
    theta1_prev = x_k.theta1;
    theta2_prev = x_k.theta2;
    tension1_prev = x_k.tension1;
    tension2_prev = x_k.tension2;

    // Get new absolute positions
    spi_change_mode(&SPI_CONF_MTR_ENCODER_A);
    x_k.theta1 = motor_encoder_get_position(&MOTOR_ENCODER_A);
    x_k.theta2 = motor_encoder_get_position(&MOTOR_ENCODER_B);
    //for(int i = 0; i < 0xFF; i++);
    spi_change_mode(&SPI_CONF_TENSION_ARM_A);
    x_k.tension1 = tension_arm_get_position(&TENSION_ARM_A);
    x_k.tension2 = tension_arm_get_position(&TENSION_ARM_B);
    
    // Use backwards euler to get absolute velocities
    float T = config->sample_period;
    x_k.theta1_dot = get_delta_angle(theta1_prev, x_k.theta1) / T;
    x_k.theta2_dot = get_delta_angle(theta2_prev, x_k.theta2) / T;
    x_k.tension1_dot = (x_k.tension1 - tension1_prev) / T;
    x_k.tension2_dot = (x_k.tension2 - tension2_prev) / T;

    // Get inc encoder values
    x_k.tape_position = roller_get_tape_position(CONTROLLER_IPS_TARGET);
    x_k.tape_speed = roller_get_ips();

    // Get torques
    float torque1 = 0;
    float torque2 = 0;
    config->controller(x_k, &torque1, &torque2);

    // Send torques to motor
    motor_unit_set_torque(&MOTOR_UNIT_A, torque1);
    motor_unit_set_torque(&MOTOR_UNIT_B, torque2);

    //stopwatch_print(1, false);
    gpio_clear_pin(DEBUG_PIN);
}

void controller_disable_motors() {
    uha_motor_driver_disable(&UHA_MTR_DRVR_CONF_A);
    uha_motor_driver_disable(&UHA_MTR_DRVR_CONF_B);
}

void controller_enable_motors() {
    uha_motor_driver_enable(&UHA_MTR_DRVR_CONF_A);
    uha_motor_driver_enable(&UHA_MTR_DRVR_CONF_B);
}

void controller_start_process() {
    timer_schedule(CONTROLLER_TIMER_ID, 500.0f, controller_run_iteration);
}

void controller_stop_process() {
    timer_deschedule(CONTROLLER_TIMER_ID);
    
    // Stop motors
    motor_unit_set_torque(&MOTOR_UNIT_A, 0.0f);
    motor_unit_set_torque(&MOTOR_UNIT_B, 0.0f);
}


