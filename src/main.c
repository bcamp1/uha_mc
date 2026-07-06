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
#include "drivers/faults.h"
#include "drivers/solenoid.h"
#include "drivers/seeprom.h"
#include "drivers/stat_tracker.h"
#include "board.h"
#include "sched.h"
#include "drivers/delay.h"
#include "drivers/inc_encoder.h"
#include "control/data_collector.h"
#include "control/movement.h"

#define FIRMWARE_VERSION "UHA MOTHERBOARD FIRMWARE v0.1"
#define FIRMWARE_AUTHOR "AUTHOR: BRANSON CAMP"
#define FIRMWARE_DATE "DATE: OCTOBER 2025"

static void enable_fpu(void);
static void init_peripherals(void);
static void parse_actions();
static void enable_motors();
static void disable_motors();
static void calibrate_motors();
static void init_motors(bool enable);

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

static void init_motors(bool enable) {
    motors_init();
    
    if (enable) {
        enable_motors();
    } else {
        disable_motors();
    }
}

static void enable_motors() {
    // Enable takeup
    uart_println("Enabling takeup...");
    RXError err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_takeup_enable();
    }
    uart_println("Takeup enabled.");

    // Enable takeup
    uart_println("Enabling supply...");
    err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_supply_enable();
    }
    uart_println("Supply enabled.");
}

static void disable_motors() {
    // Disable takeup
    uart_println("Disabling takeup...");
    RXError err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_takeup_disable();
    }
    uart_println("Takeup disabled.");

    // Disable takeup
    uart_println("Disabling supply...");
    err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_supply_disable();
    }
    uart_println("Supply disabled.");

    // Disable capstan
    uart_println("Disabling capstan...");
    err = RX_ERR_NO_DATA;
    while (err != RX_ERR_OK) {
        err = motors_capstan_disable();
    }
    uart_println("Capstan disabled.");
    
    // Disable pinch solenoid
    solenoid_pinch_disengage();
}

static void calibrate_motors() {
    // Take the 500 Hz movement tick off the air first: it shares the half-duplex
    // RS485 bus and harvests each slave's meta-fault byte every tick. If it keeps
    // running during calibration it (a) preempts our CALIB_ENCODER transactions
    // mid-frame and (b) latches the slave's transient "calibrating" meta byte,
    // which trips faults_disarm_required() -> spurious motor-fault disarm.
    timer_deschedule(ID_STATE_MACHINE_TICK);

    disable_motors();
    movement_init();
    motors_takeup_calibrate_encoder();
    motors_supply_calibrate_encoder();

    // Bus is ours alone until here; resume the tick (next harvest sees the
    // post-calibration, idle meta bytes).
    timer_schedule(ID_STATE_MACHINE_TICK, FREQUENCY_STATE_MACHINE_TICK, PRIO_STATE_MACHINE_TICK, movement_tick);
}

int main(void) {
	init_peripherals();
    uart_init();
    delay(0xFFF);
    uart_println("--------MOTOR CONTROLLER START--------");

    // Persistent storage (SmartEEPROM). On a fresh board this provisions the
    // user-page fuses and resets -- the debug LED blinks during that one-time
    // step, and halts on a fast blink if storage can't be brought up. Run early,
    // before anything that reads persistent state.
    if (seeprom_init_or_provision()) {
        uart_println("SmartEEPROM ready.");

        // Persistence test: byte 0 is a boot counter. A freshly provisioned cell
        // reads 0xFF (erased), so treat that as "never booted" -> start at 1. The
        // byte wraps after 255 boots, which is fine for testing.
        uint8_t boots = seeprom_read_byte(0);
        boots = (boots == 0xFF) ? 1 : (uint8_t)(boots + 1);
        seeprom_write_byte(0, boots);
        uart_print("Total boots: ");
        uart_println_int(boots);
    } else {
        uart_println("!! SmartEEPROM unavailable !!");
    }

    tension_init();
    inc_encoder_init();
    solenoid_pinch_init();
    //tension_init_supply_only();

    //motor_comms_init();
    //motors_init();
    //comms_init();
    command_center_init();

    init_motors(false);
    faults_init();

    gpio_init_pin(DEBUG_PIN, GPIO_DIR_OUT, GPIO_ALTERNATE_NONE);
    gpio_set_pin(DEBUG_PIN);
    data_collector_init();
    movement_init();
    delay(0xFFF);
    timer_schedule(ID_STATE_MACHINE_TICK, FREQUENCY_STATE_MACHINE_TICK, PRIO_STATE_MACHINE_TICK, movement_tick);

    // Periodically persist playback stats to EEPROM (~every 5 s, low priority).
    schedule_stat_flusher();

#if DISARM_ON_FAULT
    bool fault_disarmed = false;
#endif
    while (1) {

        //uart_print("Play Time: ");
        //uart_print_float(movement_get_playback_time());
        //uart_print(" Play Dist: ");
        //uart_println_float(movement_get_playback_travel());
        CommandCenterSimpleAction action = command_center_get_action();
        //uart_println_float(movement_get_playback_travel());

#if DISARM_ON_FAULT
        // Disarm on any active fault: drop to the same safe state as STOP and
        // latch until a clean re-arm. Use the non-blocking broadcast disable
        // (the per-motor disable_motors() retries forever, which would hang if
        // the fault is a motor that won't answer on RS485).
        if (!fault_disarmed && faults_disarm_required()) {
            fault_disarmed = true;
            movement_set_fault_disarm(true);   // tick stops driving (still polls at 0 torque)
            motors_disable_all();
            movement_init();
            uart_println("!! DISARMED: motor fault !!");
        }

        // While disarmed, the only accepted command is a re-arm: CMD_PLAY once the
        // fault has cleared. Everything else (including a still-faulted re-arm) is
        // swallowed. A clean re-arm falls through to the normal CMD_PLAY handler.
        if (fault_disarmed) {
            if (action == CMD_PLAY && !faults_disarm_required()) {
                fault_disarmed = false;
                movement_set_fault_disarm(false);
            } else {
                if (action == CMD_PLAY) uart_println("Re-arm refused: motor fault present");
                action = CMD_NONE;
            }
        }
#endif

        switch (action) {
            case CMD_STOP:
                enable_motors();
                movement_set_target_idle();
                break;
            case CMD_PLAY:
                gpio_set_pin(PIN_DEBUG1);
                gpio_clear_pin(PIN_DEBUG1);
                enable_motors();
                movement_set_target_playback();
                break;
            case CMD_FAST_FORWARD:
                movement_set_target_ff(100.0f);
                break;
            case CMD_REWIND:
                movement_set_target_rew(100.0f);
                break;
            case CMD_GOTO_LOC:
                movement_set_target_mem(command_center_get_goto_loc(), 100.0f);
                break;
            case CMD_SPOOL:
                break;
            case CMD_SET_ZERO:
                break;
            case CMD_SET_CAPSTAN_SPEED:
                break;
            case CMD_CALIBRATE:
                uart_println("CMD_CALIBRATE");
                calibrate_motors();
                break;
            case CMD_DISABLE:
                gpio_set_pin(PIN_DEBUG1);
                gpio_clear_pin(PIN_DEBUG1);
                disable_motors();
                movement_init();
                break;
            case CMD_NONE:
                break;
        }
        faults_check_health();
        movement_debug_print_on_change();
    }

}

