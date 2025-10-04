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

#define FIRMWARE_VERSION "UHA MOTHERBOARD FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

typedef enum {
    IDLE,
    FF,
    REW,
    PLAYBACK,
} State;

static State state = IDLE;

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

static void playback_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = -0.8;
    const float k_s = 0.8;

    const float r_t = 0.5f;
    const float r_s = 0.5f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;
}

static void ff_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = -0.8;
    const float k_s = 0.8;

    const float r_t = 1.0f;
    const float r_s = 0.5f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;

    *u_t -= 0.4f;
}

static void rew_controller(float tension_t, float tension_s, float* u_t, float* u_s) {
    const float k_t = -0.8;
    const float k_s = 0.8;

    const float r_t = 0.5f;
    const float r_s = 1.0f;

    float e_a = r_t - tension_t;
    float e_b = r_s - tension_s;

    *u_t = k_t * e_a;
    *u_s = k_s * e_b;
}

static void control_loop() {
    float tension_t = tension_arm_get_position(&TENSION_ARM_A);
    float tension_s = tension_arm_get_position(&TENSION_ARM_B);

    float u_t = 0.0f;
    float u_s = 0.0f;
    
    switch (state) {
        case PLAYBACK:
            playback_controller(tension_t, tension_s, &u_t, &u_s);
        break;
        case FF:
            ff_controller(tension_t, tension_s, &u_t, &u_s);
        break;
        case REW:
            rew_controller(tension_t, tension_s, &u_t, &u_s);
        break;
        default:
        break;
    }

    bldc_set_torque_float(&BLDC_CONF_TAKEUP, u_t);
    bldc_set_torque_float(&BLDC_CONF_SUPPLY, u_s);
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

    gpio_clear_pin(PIN_DEBUG1);
    gpio_clear_pin(PIN_DEBUG2);

    timer_init_all();

    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);

    bldc_init_all();
    //bldc_init(&BLDC_CONF_SUPPLY);

    delay(0x4FFFF);
    //bldc_enable(&BLDC_CONF_SUPPLY);
    //bldc_enable_all();
    bldc_disable_all();

    delay(0x8FFFF);

    bldc_set_torque_float(&BLDC_CONF_SUPPLY, 0.0f);
    bldc_set_torque_float(&BLDC_CONF_TAKEUP, 0.0f);

    //float theta = 0.0f;
    //float sin = 0.0f;
    //float cos = 0.0f;
    
    state = IDLE;
    timer_schedule(0, 500, 1, control_loop);

    while (1) {
        char user_input = uart_get();
        switch (user_input) {
            case 'p':
                uart_println("[STATE] Playback");
                state = PLAYBACK;
                bldc_enable_all();
                break;
            case 's':
                uart_println("[STATE] Idle");
                state = IDLE;
                bldc_disable_all();
                break;
            case 'f':
                uart_println("[STATE] Fast Forward");
                state = FF;
                bldc_enable_all();
                break;
            case 'r':
                uart_println("[STATE] Rewind");
                state = REW;
                bldc_enable_all();
                break;
        }
    }
}

