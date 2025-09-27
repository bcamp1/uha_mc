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
#include "foc/fast_sin_cos.h"

static void enable_fpu(void);
static void init_peripherals(void);
static void stopwatch_test();

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

    stepper_init(&STEPPER_CONF_1);
    delay(0xFFFFF);
    stepper_enable(&STEPPER_CONF_1);
    stepper_send_pulses(&STEPPER_CONF_1, stepper_steps, stepper_delay);

	while (1) {
        stepper_send_pulses(&STEPPER_CONF_1, stepper_steps, stepper_delay);
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

static void control_loop() {
    float pos_a = tension_arm_get_position(&TENSION_ARM_A);
    float pos_b = tension_arm_get_position(&TENSION_ARM_B);

    float k_a = 0.8;
    float k_b = 0.8;

    float r_a = 0.5f;
    float r_b = 0.5f;

    float e_a = r_a - pos_a;
    float e_b = r_b - pos_b;

    float u_a = k_a * e_a;
    float u_b = k_b * e_b;

    bldc_set_torque_float(&BLDC_CONF_TAKEUP, u_a);
    bldc_set_torque_float(&BLDC_CONF_SUPPLY, u_b);
}

int main(void) {
	init_peripherals();
    delay(0x4FFF);

    gpio_clear_pin(PIN_DEBUG1);
    gpio_clear_pin(PIN_DEBUG2);

    timer_init_all();

    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);

    bldc_init_all();
    //bldc_init(&BLDC_CONF_SUPPLY);

    delay(0x4FFFF);
    //bldc_enable(&BLDC_CONF_SUPPLY);
    bldc_enable_all();

    delay(0x8FFFF);

    bldc_set_torque_float(&BLDC_CONF_SUPPLY, 0.0f);
    bldc_set_torque_float(&BLDC_CONF_TAKEUP, 0.0f);

    //float theta = 0.0f;
    //float sin = 0.0f;
    //float cos = 0.0f;

    timer_schedule(0, 50, 1, control_loop);

    while (1) {
        //theta += 2.0f;
        //arm_sin_cos_f32(theta, &sin, &cos);
        //x += 5;
        //bldc_set_torque_float(&BLDC_CONF_CAPSTAN, 0.4f*sin);
        //delay(0xFFF);
        //bldc_set_torque_float(&BLDC_CONF_SUPPLY, 0.4f*sin);
        //delay(0xFFF);
        //bldc_set_torque_float(&BLDC_CONF_TAKEUP, 0.4f*sin);
        //delay(0xFFF);
    }
}

