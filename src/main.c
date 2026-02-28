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
#include "drivers/comms.h"
#include "periphs/uart.h"
#include "drivers/motor_encoder.h"
#include "drivers/tension_arm.h"
#include "drivers/stopwatch.h"
#include "board.h"
#include "sched.h"
#include "drivers/delay.h"
#include "drivers/motor_encoder.h"
#include "drivers/trq_pwm.h"
#include "drivers/bldc.h"
#include "drivers/inc_encoder.h"
#include "drivers/solenoid.h"
#include "foc/fast_sin_cos.h"
#include "control/state_machine.h"
#include "control/data_collector.h"
#include "control/movement.h"

#define FIRMWARE_VERSION "UHA MOTHERBOARD FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

static void enable_fpu(void);
static void init_peripherals(void);
static void stopwatch_test();
static void parse_actions();
static void parse_movement_actions();

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

/*
static void stopwatch_test() {
    stopwatch_init();
    bool running = false;
    rs422_println("UART Stopwatch Test (press s)");
    
    while (1) {
        //uint32_t time = stopwatch_underlying_time();
        //rs422_println_int_base(time, 16);
        if (rs422_get() == 's') {
            if (!running) {
                rs422_println("Starting stopwatch... s to stop");
                running = true;
                stopwatch_start(0);
            } else {
                float dt = ticks_to_time(stopwatch_stop(0, false));
                running = false;
                rs422_print("Stopped. Took ");
                rs422_print_float(dt);
                rs422_println(" seconds.");
            }
        }
    }
}
*/

static void encoder_test() {
    rs422_println("Starting motor encoder test");
    motor_encoder_init(&MOTOR_ENCODER_CONF);

    while (true) {
        float pos = motor_encoder_get_position(&MOTOR_ENCODER_CONF);
        rs422_println_float(pos);
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

static void comms_test() {
    comms_init();

    const uint8_t data[3] = {0x01, 0x02, 0x03};

    while (1) {
        comms_send_bytes(data, 3);
    }
}

static volatile float torque_cmd = 0.0f;

static void mock_movement_tick() {
    bldc_set_torque_float(&BLDC_CONF_TAKEUP, torque_cmd);
    bldc_set_torque_float(&BLDC_CONF_SUPPLY, torque_cmd);
}

int main(void) {
	init_peripherals();
    delay(0xFFF);

    //gpio_set_pin(PIN_DEBUG1);
    // Print firmware info
    //rs422_println("\n");
    //rs422_println("--------------------");
    //rs422_println(FIRMWARE_VERSION);
    //rs422_println(FIRMWARE_AUTHOR);
    //rs422_println(FIRMWARE_DATE);
    //rs422_println("--------------------");
    //delay(0xFFFF);
    
    // Motor control-specific peripherals
    tension_arm_init(&TENSION_ARM_A);
    tension_arm_init(&TENSION_ARM_B);

    //uart_init();
    //tension_arm_test();

    inc_encoder_init();
    bldc_init(&BLDC_CONF_TAKEUP);
    bldc_enable(&BLDC_CONF_TAKEUP);

    bldc_init(&BLDC_CONF_SUPPLY);
    bldc_enable(&BLDC_CONF_SUPPLY);

    //solenoid_pinch_init();
    
    data_collector_init();

    movement_init();
    timer_schedule(ID_STATE_MACHINE_TICK, FREQUENCY_STATE_MACHINE_TICK, PRIO_STATE_MACHINE_TICK, movement_tick);

    comms_init();
    delay(0xFF);

    uint8_t data[10];
    uint8_t data_len;
    while (1) {
        //rs422_println_float(data_collector_get_tape_position());
        if (comms_get_data(data, &data_len, 10)) {
            //torque_cmd = 0.4f;
            gpio_toggle_pin(PIN_DEBUG2);
            switch (data[0]) {
                case COMMS_CMD_ACTION_STOP:
                    //rs422_println("[ACTION] Stop");
                    movement_set_target_idle();
                    break;
                case COMMS_CMD_ACTION_PLAYBACK:
                    //rs422_println("[ACTION] Playback");
                    movement_set_target_playback();
                    break;
                case COMMS_CMD_ACTION_REWIND:
                    //rs422_println("[ACTION] Rewind");
                    movement_set_target_rew(2.0f);
                    break;
                case COMMS_CMD_ACTION_FF:
                    //rs422_println("[ACTION] Fast Forward");
                    movement_set_target_ff(2.0f);
                    break;
                case COMMS_CMD_ACTION_MEM:
                    //rs422_println("[ACTION] Go to Memory");
                    movement_set_target_mem(120.0f, 2.0f);
                    break;
                case COMMS_CMD_ACTION_RTZ:
                    //rs422_println("[ACTION] Go to Memory");
                    movement_set_target_mem(120.0f, 2.0f);
                    break;
            }
        }
        
        comms_send_float(COMMS_CMD_TRANSMIT_TAPE_POS, data_collector_get_tape_position());
        delay(0xFFF);
    }
}

// Depracated
void parse_movement_actions() {
    //rs422_println("Parsing");
    int16_t user_input = rs422_get();
    switch (user_input) {
        case 'p':
            rs422_println("[ACTION] Playback");
            movement_set_target_playback();
            break;
        case 's':
            rs422_println("[ACTION] Stop");
            movement_set_target_idle();
            break;
        case 'f':
            rs422_println("[ACTION] Fast Forward");
            movement_set_target_ff(2.0f);
            break;
        case 'r':
            rs422_println("[ACTION] Rewind");
            movement_set_target_rew(2.0f);
            break;
        case 'm':
            rs422_println("[ACTION] Go to Memory");
            movement_set_target_mem(120.0f, 2.0f);
    }
    delay(0x1FFF);
}

