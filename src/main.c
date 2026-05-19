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

int main(void) {
	init_peripherals();
    uart_init();
    tension_init();
    //inc_encoder_init();
    //tension_init_supply_only();

    //motor_comms_init();
    //motors_init();
    //comms_init();
    command_center_init();

    gpio_init_pin(DEBUG_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_set_pin(DEBUG_PIN);
    //data_collector_init();
    //movement_init();
    //delay(0xFFF);
    //timer_schedule(ID_STATE_MACHINE_TICK, FREQUENCY_STATE_MACHINE_TICK / 100.0f, PRIO_STATE_MACHINE_TICK, movement_tick);

    uart_println("Motor Controller Start!");

    while (1) {
        //uart_println_float(tension_get_takeup());
        //delay(0xFFFF);
        //CommsRxResult rx = comms_get_data();
        //if (rx.err == RX_ERR_OK && rx.data_len >= 1) {
        //    comms_send_bytes(&rx.data[0], 1);
        //}
        uart_print_float(tension_get_supply());
        uart_print(" ");
        uart_println_float(tension_get_takeup());
        delay(0xFFF);
        //uart_println_float(2.0f);
    }
        /*
        x += 0.00003f;
        if (x > 1.0f) x = 0.0f;

        gpio_set_pin(PIN_DEBUG1);
        gpio_clear_pin(PIN_DEBUG1);
        //motors_set_reel_torques(-x, x);
        
        uart_print("Takeup -> ");
        err = motors_takeup_get_faults(&faults);
        motor_comms_print_error(err);
        uart_print(": ");
        uart_println_int(faults);
        delay(0xFFF);

        uart_print("Supply -> ");
        err = motors_supply_get_faults(&faults);
        motor_comms_print_error(err);
        uart_print(": ");
        uart_println_int(faults);
        delay(0xFFF);
        // read_test(0x2, x);
        // x++;
        // read_test(0x4, x);
        // x++;
        */
    //}
        //data_collector_update();
        //gpio_toggle_pin(PIN_DEBUG1);
        //gpio_toggle_pin(PIN_DEBUG2);
        //uart_print_float(data_collector_get_takeup_tension());
        //uart_print(" ");
        //uart_print_float(data_collector_get_supply_tension());
        //uart_print(" ");
        //uart_print_float(data_collector_get_tape_position());
        //uart_print(" ");
        //uart_println_float(data_collector_get_tape_speed());
        //delay(0xFFF);
    //}

    //gpio_set_pin(PIN_DEBUG1);

    // Motor control-specific peripherals
    //tension_arm_init(&TENSION_ARM_A);
    //tension_arm_init(&TENSION_ARM_B);

    //uart_init();
    //tension_arm_test();

    //inc_encoder_init();
    //bldc_init(&BLDC_CONF_TAKEUP);
    //bldc_enable(&BLDC_CONF_TAKEUP);

    //bldc_init(&BLDC_CONF_SUPPLY);
    //bldc_enable(&BLDC_CONF_SUPPLY);

    //solenoid_pinch_init();
    
    //data_collector_init();

    //movement_init();
    //timer_schedule(ID_STATE_MACHINE_TICK, FREQUENCY_STATE_MACHINE_TICK, PRIO_STATE_MACHINE_TICK, movement_tick);

    //comms_init();
    //delay(0xFF);

}

