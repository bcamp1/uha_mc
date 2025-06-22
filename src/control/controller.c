#include "controller.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "../drivers/motor_unit.h"
#include "../drivers/uha_motor_driver.h"
#include "../drivers/motor_encoder.h"
#include "../drivers/tension_arm.h"
#include "../drivers/stopwatch.h"
#include "../drivers/roller.h"
#include "../drivers/board.h"
#include "../periphs/gpio.h"
#include "../periphs/uart.h"
#include "../periphs/spi.h"
#include "../periphs/timer.h"
#include "sensor_state.h"

static const float sample_rate = 500.0f;

static volatile ControllerConfig* config = NULL;

static volatile uint32_t timesteps = 0;
static volatile float time;

static volatile SensorState x_k = {0.0f};
static volatile SensorState x_kminus1 = {0.0f};
static volatile SensorState x_kminus2 = {0.0f};
static volatile SensorState integrator = {0.0f};
static volatile SensorState e_x = {0.0f};
static volatile SensorState e_v = {0.0f};
static volatile SensorState e_a = {0.0f};
static const SensorState zeros = {0,0,0,0,0,0};

static volatile bool motors_enabled = true;

static SensorState measure_sensor_state() {

    // Get new absolute positions
    spi_change_mode(&SPI_CONF_MTR_ENCODER_A);
    float theta1 = motor_encoder_get_position(&MOTOR_ENCODER_A);
    float theta2 = motor_encoder_get_position(&MOTOR_ENCODER_B);
                                    
    spi_change_mode(&SPI_CONF_TENSION_ARM_A);
    float tension1 = tension_arm_get_position(&TENSION_ARM_A);
    float tension2 = tension_arm_get_position(&TENSION_ARM_B);

    float tape_position = roller_get_tape_position(15.0f);
    float tape_speed = roller_get_ips();

    return (SensorState) {
       .state = {theta1, theta2, tape_position, tape_speed, tension1, tension2}, 
    };
}

void controller_send_state_uart() {
    float data[7] = {
        time,
        x_k.state[0],
        x_k.state[1],
        x_k.state[2],
        x_k.state[3],
        //x_k.state[4],
        //x_k.state[5],
        e_v.state[0],
        e_v.state[1],
    };
    //uart_send_float_arr(data, 7);
    uart_println_float_arr(data, 7);
}

void controller_print_state() {
    float data[7] = {
        time,
        x_k.state[0],
        x_k.state[1],
        x_k.state[2],
        x_k.state[3],
        x_k.state[4],
        x_k.state[5],
    };
    uart_println_float_arr(x_k.state, 6);
}

void controller_init_all_hardware() {
	// Init hardware
    motor_unit_init(&MOTOR_UNIT_A);
	motor_unit_init(&MOTOR_UNIT_B);
    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);
    roller_init(); 
}

void controller_set_config(ControllerConfig* c) {
    config = c;
}

void controller_run_iteration() {
    gpio_set_pin(LED_PIN);
    // Get sensor states
    x_kminus2 = x_kminus1;
    x_kminus1 = x_k;
    x_k = measure_sensor_state(); 

    // Calculate v_k and a_k using backwards approx
    SensorState v_k = sensor_state_scale(sensor_state_sub(x_k, x_kminus1), sample_rate);
    
    // Acceleration
    SensorState twox_kminus1 = sensor_state_scale(x_kminus1, 2);
    SensorState a_k = sensor_state_scale(sensor_state_add(sensor_state_sub(x_k, twox_kminus1), x_kminus2), sample_rate*sample_rate);
    
    // Calculate errors
    SensorState r = (SensorState) {
       .state = {
           0.0f, 
           0.0f, 
           0.0f, 
           config->reference->tape_speed,
           config->reference->tension1,
           config->reference->tension2,
       } 
    };

    SensorState r_dot = (SensorState) {
       .state = {
           config->reference->theta1_dot,
           config->reference->theta2_dot,
           0.0f,
           0.0f,
           0.0f,
           0.0f,
       } 
    };

    e_x = sensor_state_sub(r, x_k);
    e_v = sensor_state_sub_raw(r_dot, v_k);
    e_a = sensor_state_scale(a_k, -1.0f);

    // Update integrator
    if (motors_enabled) {
        integrator = sensor_state_add(integrator, sensor_state_scale(e_x, (1.0f/sample_rate)));
    } else {
        integrator = zeros; 
    }

    // Get torques
    float torque1 = 0;
    float torque2 = 0;
    config->controller(e_x, e_v, e_a, integrator, &torque1, &torque2);

    // Send torques to motor
    motor_unit_set_torque(&MOTOR_UNIT_A, torque1);
    motor_unit_set_torque(&MOTOR_UNIT_B, torque2);

    // Update time
    time = controller_get_time();
    timesteps++;
    gpio_clear_pin(LED_PIN);
}

float controller_get_time() {
    return (1.0f / sample_rate) * (float) timesteps;
}

void controller_disable_motors() {
    uha_motor_driver_disable(&UHA_MTR_DRVR_CONF_A);
    uha_motor_driver_disable(&UHA_MTR_DRVR_CONF_B);
    motors_enabled = false;
}

void controller_enable_motors() {
    uha_motor_driver_enable(&UHA_MTR_DRVR_CONF_A);
    uha_motor_driver_enable(&UHA_MTR_DRVR_CONF_B);
    motors_enabled = true;
}

void controller_start_process() {
    timesteps = 0;
    timer_schedule(CONTROLLER_TIMER_ID, sample_rate, controller_run_iteration);
}

void controller_stop_process() {
    timer_deschedule(CONTROLLER_TIMER_ID);
    
    // Stop motors
    motor_unit_set_torque(&MOTOR_UNIT_A, 0.0f);
    motor_unit_set_torque(&MOTOR_UNIT_B, 0.0f);
}

