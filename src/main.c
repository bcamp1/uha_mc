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
#include "control/controller_tests.h"
#include "drivers/inc_encoder.h"
#include "drivers/stopwatch.h"

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

    // Incremental Encoder
	inc_encoder_init();
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

static void stopwatch_test(void) {
    stopwatch_init();
    bool running = false;
    uart_println("UART Stopwatch Test (press s)");
    
    while (1) {
        uint32_t time = stopwatch_underlying_time();
        //uart_println_int_base(time, 16);
        if (uart_get() == 's') {
            if (!running) {
                uart_println("Starting stopwatch... s to stop");
                running = true;
                stopwatch_start(0);
            } else {
                float dt = stopwatch_stop(0, false);
                running = false;
                uart_print("Stopped. Took ");
                uart_print_float(dt);
                uart_println(" seconds.");
            }
        }
    }
}

int main(void) {
	// Print welcome message
	init_peripherals();
	print_welcome();
    //stopwatch_test();

	bool send_logs = true;
	bool uart_toggle = true;
	bool start_on = false;
	//controller_tests_run(&controller_config_demo, send_logs, uart_toggle, start_on);
	while (1) {
        //x = stopwatch_underlying_time();
        //uart_print(".");
		float pos = inc_encoder_get_pos();
		float vel = inc_encoder_get_vel();
        uint32_t ticks = inc_encoder_get_dt_ticks();
        float data[3] = {pos, vel, (float) ticks};
        uart_send_float_arr(data, 3);
        //uart_print_float(pos);
        //uart_print(" ");
        //uart_print_float(vel);
        //uart_print(" ");
        //uart_println_float(ticks);
	}
}
