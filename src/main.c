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
#include "periphs/uart.h"
#include "drivers/motor_encoder.h"
#include "drivers/tension_arm.h"
#include "drivers/stopwatch.h"
#include "board.h"
#include "drivers/delay.h"
#include "drivers/motor_encoder.h"
#include "drivers/stepper.h"
#include "drivers/trq_pwm.h"
#include "drivers/bldc.h"
#include "drivers/inc_encoder.h"
#include "drivers/solenoid.h"
#include "foc/fast_sin_cos.h"
#include "control/state_machine.h"

#define FIRMWARE_VERSION "UHA MOTHERBOARD FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

static void enable_fpu(void);
static void init_peripherals(void);
static void stopwatch_test();
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
	
	// Init the UART
	uart_init();
    
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

static void encoder_test() {
    uart_println("Starting motor encoder test");
    motor_encoder_init(&MOTOR_ENCODER_CONF);

    while (true) {
        float pos = motor_encoder_get_position(&MOTOR_ENCODER_CONF);
        uart_println_float(pos);
    }
}

static void stepper_test() {
    int32_t stepper_steps = 200;
    uint32_t stepper_delay = 0x4FF;

    const StepperConfig* step = &STEPPER_CONF_CAPSTAN;

    stepper_init(step);
    delay(0xFFFFF);
    stepper_enable(step);
    stepper_send_pulses(step, stepper_steps, stepper_delay);

	while (1) {
        stepper_send_pulses(step, stepper_steps, stepper_delay);
        stepper_steps *= -1;
	}
}

static void tension_arm_test() {
    while (true) {
        float pos_a = tension_arm_get_position(&TENSION_ARM_A);
        float pos_b = tension_arm_get_position(&TENSION_ARM_B);
        float data[2] = {pos_a, pos_b};
        uart_println_float_arr(data, 2);
    }
}

int main(void) {
	init_peripherals();
    delay(0xFFF);

    // Print firmware info
    uart_println("\n");
    uart_println("--------------------");
    uart_println(FIRMWARE_VERSION);
    uart_println(FIRMWARE_AUTHOR);
    uart_println(FIRMWARE_DATE);
    uart_println("--------------------");
    delay(0xFFFF);



    //float theta = 0.0f;
    //float sin = 0.0f;
    //float cos = 0.0f;
    
    state_machine_init();
    //gpio_init_pin(PIN_ROLLER_PULSE, GPIO_DIR_IN, GPIO_ALTERNATE_NONE);
    //
    inc_encoder_init();
    timer_schedule(0, STATE_MACHINE_FREQUENCY, 1, state_machine_tick);
    //tension_arm_test();
    //

    bool engaged = false;
    while (1) {
        //uart_println_int(inc_encoder_get_ticks());
        uart_println_float(inc_encoder_get_position());
        //uart_println_float(state_machine_get_tape_speed());
        parse_actions();
    }
}

void parse_actions() {
    //uart_println("Parsing");
    char user_input = uart_get();
    switch (user_input) {
        case 'p':
            uart_println("[ACTION] Playback");
            bldc_enable_all();
            state_machine_take_action(PLAY_ACTION);
            break;
        case 's':
            uart_println("[ACTION] Stop");
            bldc_disable_all();
            state_machine_take_action(STOP_ACTION);
            break;
        case 'f':
            uart_println("[ACTION] Fast Forward");
            bldc_enable_all();
            state_machine_take_action(FF_ACTION);
            break;
        case 'r':
            uart_println("[ACTION] Rewind");
            bldc_enable_all();
            state_machine_take_action(REW_ACTION);
            break;
    }
    /*
    uart_print_float(state_machine_get_supply_speed());
    uart_print(" ");
    uart_println_float(state_machine_get_takeup_speed());
    */
    if (gpio_get_pin(PIN_ROLLER_PULSE)) {
        gpio_clear_pin(PIN_DEBUG2);
    } else {
        gpio_set_pin(PIN_DEBUG2);
    }
    delay(0x1FFF);
}

