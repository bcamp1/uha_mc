/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include <sam.h>
#include "periphs/gpio.h"
#include "periphs/spi.h"
#include "periphs/spi_async.h"
#include "periphs/clocks.h"
#include "periphs/uart.h"
#include "periphs/timer.h"
#include "control/controller_tests.h"
#include "control/controller.h"
#include "drivers/roller.h"
#include "drivers/motor_unit.h"
#include "drivers/uha_motor_driver.h"
#include "drivers/motor_encoder.h"
#include "drivers/tension_arm.h"
#include "drivers/stopwatch.h"
#include "drivers/delay.h"
#include "drivers/board.h"
#include "drivers/spi_collector.h"

static void enable_fpu(void);
static void init_peripherals(void);
static void print_welcome(void);

static void init_peripherals(void) {
	// Init clock to use 32K OSC in closed-loop 48MHz mode
    // Then its boosted to 120MHz
	wntr_system_clocks_init();
	
	// Enable floating-point unit
	enable_fpu();

	// Init useful debugging GPIO pins
	gpio_init_pin(LED_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
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
	controller_tests_run(&controller_tests_config, send_logs, uart_toggle, start_on);
}

static uint8_t spi_read_bytes[2] = {0, 0};
static uint8_t spi_write_bytes[2] = {0, 0};

static void encoder_test() {
    //NVIC_SetPriorityGrouping(0);
    spi_collector_init();
    spi_collector_set_torque_a(0.4f);
    spi_collector_set_torque_b(-0.0f);
    spi_collector_enable_service();
    while (1) {
        //gpio_set_pin(DEBUG_PIN);
        /*
        float data[6] = {
            spi_collector_get_encoder_a(),
            spi_collector_get_encoder_a_pole(),
            spi_collector_get_encoder_b(),
            spi_collector_get_encoder_b_pole(),
            spi_collector_get_tension_a(),
            spi_collector_get_tension_b(),
        };

        uart_println_float_arr(data, 6);
        */
    }
    /*
       while (1) {
       float a = motor_encoder_get_pole_position(&MOTOR_ENCODER_A); 
       float b = motor_encoder_get_pole_position(&MOTOR_ENCODER_B); 
       uart_print_float(a);
       uart_put(' ');
       uart_println_float(b);
       }
       */
}

int main(void) {
	init_peripherals();
	print_welcome();
    //timer_schedule(1, 500.0f, timer_test);
    delay(0x4FFF);
    uart_println("\nStarting Controller Test");
    controller_test();
    //delay(0x4FFF);
    //uart_println("\nStarting Encoder Test");
    //encoder_test();

	while (1) {
        delay(0x4FF);
        uart_println("Hello world");
		//float ips = roller_get_ips();
		//float tape_pos = roller_get_tape_position(15.0f);
        //float data[2] = {ips, tape_pos};
        //uart_println_float_arr(data, 2);
	}
}

