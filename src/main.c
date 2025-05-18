/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include "sam.h"
#include "periphs/gpio.h"
#include "periphs/clocks.h"
#include "periphs/uart.h"
#include "control/controller_tests.h"

#include "foc/foc_math_fpu.h"
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

int main(void) {
	// Print welcome message
	init_peripherals();
	print_welcome();

	controller_tests_run(&controller_config_demo, false, true, false);

	while (1) {
		uart_print("MAINLOOP");
	}
}
