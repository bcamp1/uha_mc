/*
 * MotorControllerBoard.c
 *
 * Created: 3/9/2025 3:26:48 PM
 * Author : brans
 */ 

#include "sam.h"
#include "periphs/gpio.h"
#include "periphs/pwm.h"
#include "drivers/uha_motor_driver.h"
#include "periphs/spi.h"
#include "periphs/clocks.h"
#include "periphs/uart.h"
#include "drivers/tension_arm.h"
#include "drivers/inc_encoder.h"
#include "drivers/motor_encoder.h"
#include "periphs/timer.h"
#include "drivers/motor_unit.h"
#include "control/controller.h"

#include "foc/foc_math_fpu.h"

#define LED PIN_PA15
#define DEBUG_PIN PIN_PA14

static void tension_arm_test();
static void inc_encoder_test();
static void motor_test();
static void motor_test2();
static void motor_test3();
static void blink_test();
static void print_tension_info();
static void print_encoder_info();
static float force_function_1(float t, float theta1, float theta2);
static float force_function_2(float t, float theta1, float theta2);
static float controller_func_1(State x);
static float controller_func_2(State x);

void enable_fpu(void) {
	// Enable CP10 and CP11 (FPU coprocessors)
	SCB->CPACR |= (0xF << 20);

	// Ensure all memory accesses complete before continuing
	__DSB();  // Data Synchronization Barrier
	__ISB();  // Instruction Synchronization Barrier
}

int main(void) {
	wntr_system_clocks_init();
	enable_fpu();

	// init gpio
	gpio_init_pin(LED, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	gpio_init_pin(DEBUG_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
	
	uart_init();
	uart_println("Starting up...");

	motor_test3();
	//tension_arm_test();
	uint16_t x = 0;
	while (1) {
		x--;
		for (int i = 0; i < 0x1; i++);
		gpio_toggle_pin(LED);
		//uart_println("TEST");
		uart_println_float((float) x);
	}
}


static void inc_encoder_test() {
	inc_encoder_init();
	
	while (1) {
		float encoder_vel = inc_encoder_get_vel();
		uart_println_float(encoder_vel);
	}
}

static void blink_test() {
	while (1) {
		for (int i = 0; i < 0xFFFFF; i++) {}
		uart_println("Blink!");
		gpio_toggle_pin(LED);
	}
}

static void tension_arm_test() {
	tension_arm_init(&TENSION_ARM_A);
	tension_arm_init(&TENSION_ARM_B);

	while (1)
	{
		for (int i = 0; i < 0xFFF; i++) {
			gpio_toggle_pin(LED);
            print_tension_info();
		}
		
	}
}

static void motor_test() {
	//motor_unit_init(&MOTOR_UNIT_A);
	//motor_unit_init(&MOTOR_UNIT_B);
	motor_encoder_init(&MOTOR_ENCODER_A);
	motor_encoder_init(&MOTOR_ENCODER_B);
	while (1) {
		float encoder_pos_a = motor_encoder_get_position(&MOTOR_ENCODER_A);
		float encoder_pos_b = motor_encoder_get_position(&MOTOR_ENCODER_B);
		//for (int i = 0; i < 0xFFF8; i++);
		//float encoder_pos_b = motor_encoder_get_position(&MOTOR_ENCODER_B);
		uart_print("ENC A: ");
		uart_print_float(encoder_pos_a);
		uart_print(", ENC B: ");
		uart_println_float(encoder_pos_b);
		//uha_motor_driver_goto_theta(&UHA_MTR_DRVR_CONF_B, encoder_pos + 1.0f);
		//motor_unit_set_torque(&MOTOR_UNIT_B, 1.0f);
	}
}

static void print_tension_info() {
    uart_print("Tension Arm A: ");
    uart_print_float(tension_arm_get_position(&TENSION_ARM_A));
    uart_print(", Tension Arm B: ");
    uart_println_float(tension_arm_get_position(&TENSION_ARM_B));
}

static void print_encoder_info() {
	float encoder_pos_a = motor_encoder_get_pole_position(&MOTOR_ENCODER_A);
	float encoder_pos_b = motor_encoder_get_pole_position(&MOTOR_ENCODER_B);
	//for (int i = 0; i < 0xFFF8; i++);
	//float encoder_pos_b = motor_encoder_get_position(&MOTOR_ENCODER_B);
	uart_print("ENC A: ");
	uart_print_float(encoder_pos_a);
	uart_print(", ENC B: ");
	uart_println_float(encoder_pos_b);
}

static float force_function_1(float t, float theta1, float theta2) {
	float delta2 = theta2 - PI;
	float k = 0.5f;
	return k*delta2;
}

static float force_function_2(float t, float theta1, float theta2) {
	float delta2 = theta2 - PI;
	float k = 0.5f;
	return -k*delta2;
}

static void motor_test2() {
	bool enabled = false;
	uart_println("Motor Test 2 (press e to start)");
	motor_unit_init(&MOTOR_UNIT_A);
	motor_unit_init(&MOTOR_UNIT_B);
	uha_motor_driver_disable(&UHA_MTR_DRVR_CONF_A);
	uha_motor_driver_disable(&UHA_MTR_DRVR_CONF_B);
	//tension_arm_init(&TENSION_ARM_A);
	//uha_motor_driver_disable(&UHA_MTR_DRVR_CONF_B);
	float t = 0.0f;
	
	while (1) {
		t += 0.002f;
		float theta1 = motor_encoder_get_position(&MOTOR_ENCODER_A);
		float theta2 = motor_encoder_get_position(&MOTOR_ENCODER_B);
		float torque_a = force_function_1(t, theta1, theta2);
		float torque_b = force_function_2(t, theta1, theta2);
		//uart_println_float(torque_b);
		//float tension_pos = tension_arm_get_position(&TENSION_ARM_A);
		//uart_println_float(tension_pos);
		motor_unit_set_torque(&MOTOR_UNIT_A, torque_a);
		motor_unit_set_torque(&MOTOR_UNIT_B, torque_b);
		//motor_unit_energize_coils(&MOTOR_UNIT_A, 0.1f, 0.0f, 0.0f);
		//motor_unit_energize_coils(&MOTOR_UNIT_B, 0.1f, 0.0f, 0.0f);
		//print_encoder_info();
		if (uart_get() == 'e') {
			enabled = !enabled;
			if (enabled) {
				uart_println("Motors enabled. Press e to disable.");
			} else {
				uart_println("Motors disabled. Press e to enable.");
			}
			uha_motor_driver_toggle(&UHA_MTR_DRVR_CONF_A);
			uha_motor_driver_toggle(&UHA_MTR_DRVR_CONF_B);
		}
		// Basic feedback law
		//float r = 0.5f;
		//float e = tension_pos - r;
		//float u = 1.0f * e;
		//motor_unit_set_torque(&MOTOR_UNIT_B, u);
	}
}

static float controller_func_1(State x) {
	float theta1 = x.theta1;
	float theta2 = x.theta2;
	return force_function_1(0, theta1, theta2);
}

static float controller_func_2(State x) {
	float theta1 = x.theta1;
	float theta2 = x.theta2;
	return force_function_2(0, theta1, theta2);
}

static void motor_test3() {
	bool enabled = true;
	uart_println("Starting Motor Test 3 (press e to start)");
	// Create controller config
	ControllerConfig controller_config = {
		.controller1 = controller_func_1,
		.controller2 = controller_func_2,
		.sample_period = 0.01f,
	};

	controller_init_all_hardware();
	controller_set_config(&controller_config);
	
	while (1) {
		//gpio_toggle_pin(LED);
		//gpio_toggle_pin(DEBUG_PIN);
		//uart_println("01234567890123456789");
		//uart_put(0x04);
		//uart_put('\n');
		//uart_put(0x02);
		//uart_send_float(2.0213f);
        //uart_print_float(tension_arm_get_position(&TENSION_ARM_B));
        //uart_print(", ");
		controller_run_iteration();
        //print_tension_info();
		controller_send_state_uart();
		/*
		if (uart_get() == 'e') {
			enabled = !enabled;
			if (enabled) {
				uart_println("Motors enabled. Press e to disable.");
			} else {
				uart_println("Motors disabled. Press e to enable.");
			}
			uha_motor_driver_toggle(&UHA_MTR_DRVR_CONF_A);
			uha_motor_driver_toggle(&UHA_MTR_DRVR_CONF_B);
		}
			*/
	}
}
		
