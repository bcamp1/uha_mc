/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include <sam.h>
#include "periphs/gpio.h"
#include "periphs/clocks.h"
#include "periphs/uart.h"
#include "periphs/timer.h"
#include "control/controller_tests.h"
#include "drivers/roller.h"
#include "drivers/stopwatch.h"
#include "drivers/delay.h"

#define LED PIN_PA15
#define DEBUG_PIN PIN_PA14

static void enable_fpu(void);
static void init_peripherals(void);
static void print_welcome(void);

static void init_peripherals(void) {
	// Init clock to use 32K OSC in closed-loop 48MHz mode
	wntr_system_clocks_init();
	
	// Enable floating-point unit
	enable_fpu();

	// Init useful debugging GPIO pins
	gpio_init_pin(LED, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(DEBUG_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	
	// Init the UART
	uart_init();
    
    // Stop Watch
    stopwatch_init();
}

static void enable_fpu(void) {
	// Enable CP10 and CP11 (FPU coprocessors)
	SCB->CPACR |= (0xF << 20);
	// Ensure all memory accesses complete before continuing
	__DSB();  // Data Synchronization Barrier
	__ISB();  // Instruction Synchronization Barrier
}

static void print_welcome(void) {
	uart_println("---UHA MOTOR DRIVER---");
	uart_println("Author: Branson Camp");
	uart_println("Date: 5-18-24");
	uart_println("----------------------");
}

static void stopwatch_test() {
    stopwatch_init();
    bool running = false;
    uart_println("UART Stopwatch Test (press s)");
    
    while (1) {
        //uint32_t time = stopwatch_underlying_time();
        //uart_println_int_base(time, 16);
        if (uart_get() == 's') {
            if (!running) {
                uart_println("Starting stopwatch... s to stop");
                running = true;
                stopwatch_start(0);
            } else {
                float dt = ticks_to_time(stopwatch_stop(0, false));
                running = false;
                uart_print("Stopped. Took ");
                uart_print_float(dt);
                uart_println(" seconds.");
            }
        }
    }
}

static void controller_test() {
	bool send_logs = true;
	bool uart_toggle = true;
	bool start_on = false;
	controller_tests_run(&controller_config_pid, send_logs, uart_toggle, start_on);
}

static void timer_test() {
    gpio_toggle_pin(DEBUG_PIN);
}

int main(void) {
	init_peripherals();
	print_welcome();
    //timer_schedule(1, 500.0f, timer_test);
    controller_test();

	while (1) {
        delay(0x4FFF);
		float ips = roller_get_ips();
		float tape_pos = roller_get_tape_position(15.0f);
        float data[2] = {ips, tape_pos};
        uart_println_float_arr(data, 2);
	}
}

