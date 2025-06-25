#include "controller.h"
#include <stdlib.h>
#include <stdbool.h>
#include <math.h>

#include "../drivers/motor_unit.h"
#include "../drivers/uha_motor_driver.h"
#include "../drivers/motor_encoder.h"
#include "../drivers/inc_encoder.h"
#include "../drivers/tension_arm.h"
#include "../drivers/stopwatch.h"
#include "../drivers/roller.h"
#include "../drivers/board.h"
#include "../periphs/gpio.h"
#include "../periphs/uart.h"
#include "../periphs/spi.h"
#include "../periphs/timer.h"
#include "simple_filter.h"
#include "sensor_state.h"
#include "control_state.h"

static const float sample_rate = 500.0f;

static volatile ControllerConfig* config = NULL;

static volatile uint32_t timesteps = 0;
static volatile float time;

static volatile bool motors_enabled = true;

static ControlState control_state;

// Filters
static SimpleFilter takeup_reel_speed_filter        = {0.995f, 0.0f};
static SimpleFilter takeup_reel_acceleration_filter = {0.995f, 0.0f};
static SimpleFilter takeup_tension_speed_filter     = {0.99f, 0.0f};
static SimpleFilter supply_tension_speed_filter     = {0.99f, 0.0f};
static SimpleFilter supply_reel_speed_filter        = {0.9f, 0.0f};
static SimpleFilter supply_reel_acceleration_filter = {0.99f, 0.0f};
static SimpleFilter tape_speed_filter               = {0.95f, 0.0f};
static SimpleFilter tape_acceleration_filter        = {0.99f, 0.0f};


static const ControlStateFilter control_state_filter = {
    .takeup_reel_speed = &takeup_reel_speed_filter,
    .takeup_reel_acceleration = &takeup_reel_acceleration_filter,
    .takeup_tension_speed = &takeup_tension_speed_filter,

    .supply_reel_speed = &supply_reel_speed_filter,
    .supply_reel_acceleration = &supply_reel_acceleration_filter,
    .supply_tension_speed = &supply_tension_speed_filter,

    .tape_speed = &tape_speed_filter,
    .tape_acceleration = &tape_acceleration_filter,
};


void controller_send_state_uart() {
    control_state_transmit_uart(controller_get_time(), control_state);
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

    // Get new control state
    control_state = control_state_get_filtered_state(&control_state_filter, sample_rate);

    // Get error
    ControlState error_state = control_state_sub(config->reference, &control_state);

    // Get torques
    float torque1 = 0;
    float torque2 = 0;
    config->controller(error_state, &torque1, &torque2);

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

