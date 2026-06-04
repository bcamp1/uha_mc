/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include <sam.h>
#include "periphs/gpio.h"
#include "periphs/clocks.h"
#include "periphs/timer.h"
#include "drivers/rs422.h"
#include "drivers/motor_comms.h"
#include "drivers/motors.h"
#include "drivers/user_comms.h"
#include "periphs/uart.h"
#include "drivers/tension.h"
#include "drivers/stopwatch.h"
#include "drivers/command_center.h"
#include "board.h"
#include "sched.h"
#include "drivers/delay.h"
#include "drivers/inc_encoder.h"
#include "control/state_machine.h"
#include "control/data_collector.h"
#include "control/movement.h"

#define FIRMWARE_VERSION "UHA MOTHERBOARD FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

static void enable_fpu(void);
static void init_peripherals(void);
static void parse_actions();
static void enable_motors();
static void disable_motors();
static void init_motors(bool enable);

static void init_peripherals(void) {
	// Init clock to use 32K OSC in closed-loop 48MHz mode
    // Then its boosted to 120MHz
	wntr_system_clocks_init();
	
	// Enable floating-point unit
	enable_fpu();

	// Init useful debugging GPIO pins
	//gpio_init_pin(LED_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(PIN_DEBUG1, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(PIN_DEBUG2, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_clear_pin(PIN_DEBUG1);
    gpio_clear_pin(PIN_DEBUG2);
    
    // Stop Watch
    stopwatch_init();
    
    // Timers
    timer_init_all();
}

static void enable_fpu(void) {
	// Enable CP10 and CP11 (FPU coprocessors)
	SCB->CPACR |= (0xF << 20);
	// Ensure all memory accesses complete before continuing
	__DSB();  // Data Synchronization Barrier
	__ISB();  // Instruction Synchronization Barrier
}

static void comms_test() {
    comms_init();

    const uint8_t data[3] = {0x01, 0x02, 0x03};

    while (1) {
        comms_send_bytes(data, 3);
    }
}

static volatile float torque_cmd = 0.0f;

static void mock_movement_tick() {
    //bldc_set_torque_float(&BLDC_CONF_TAKEUP, torque_cmd);
    //bldc_set_torque_float(&BLDC_CONF_SUPPLY, torque_cmd);
}


static void read_test(uint8_t addr, uint8_t data) {
    static uint8_t buf[10];
    uint8_t data_len;
    RXError err = motor_comms_read(addr, data, buf, &data_len, 10); 
    uart_print("@");
    uart_print_int(addr);
    uart_print(": sent ");
    uart_print_int(data);
    uart_print(" got ");
    if (err == RX_ERR_OK) {
        uart_println_int(buf[0]); 
    } else {
        motor_comms_println_error(err);
    }
}

static void init_motors(bool enable) {
    motors_init();
    
    if (enable) {
        enable_motors();
    } else {
        disable_motors();
    }
}

static void enable_motors() {
    // Enable takeup
    uart_println("Enabling takeup...");
    RXError err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_takeup_enable();
    }
    uart_println("Takeup enabled.");

    // Enable takeup
    uart_println("Enabling supply...");
    err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_supply_enable();
    }
    uart_println("Supply enabled.");
}

static void disable_motors() {
    // Enable takeup
    uart_println("Disabling takeup...");
    RXError err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_takeup_disable();
    }
    uart_println("Takeup disabled.");

    // Enable takeup
    uart_println("Disabling supply...");
    err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_supply_disable();
    }
    uart_println("Supply disabled.");
}

int main(void) {
	init_peripherals();
    uart_init();
    delay(0xFFF);
    uart_println("--------MOTOR CONTROLLER START--------");

    tension_init();
    inc_encoder_init();
    //tension_init_supply_only();

    //motor_comms_init();
    //motors_init();
    //comms_init();
    command_center_init();

    init_motors(false);

    gpio_init_pin(DEBUG_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_set_pin(DEBUG_PIN);
    data_collector_init();
    movement_init();
    delay(0xFFF);
    timer_schedule(ID_STATE_MACHINE_TICK, FREQUENCY_STATE_MACHINE_TICK, PRIO_STATE_MACHINE_TICK, movement_tick);

    while (1) {
        CommandCenterSimpleAction action = command_center_get_action();
        switch (action) {
            case CMD_STOP:
                gpio_set_pin(PIN_DEBUG1);
                gpio_clear_pin(PIN_DEBUG1);
                disable_motors();
                movement_init();
                break;
            case CMD_PLAY:
                gpio_set_pin(PIN_DEBUG1);
                gpio_clear_pin(PIN_DEBUG1);
                enable_motors();
                movement_set_target_idle();
                break;
            case CMD_FAST_FORWARD:
                movement_set_target_ff(100.0f);
                break;
            case CMD_REWIND:
                movement_set_target_rew(100.0f);
                break;
            case CMD_GOTO_LOC:
                movement_set_target_mem(command_center_get_goto_loc(), 100.0f);
                break;
            case CMD_SPOOL:
                break;
            case CMD_SET_ZERO:
                break;
            case CMD_SET_CAPSTAN_SPEED:
                break;
            case CMD_NONE:
                break;
        }
        movement_debug_print_on_change();
    }

}

