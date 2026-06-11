/*
 * faults.c
 *
 * uha_mother fault framework. See faults.h.
 */

#include "faults.h"
#include "command_center.h"
#include "tension.h"
#include "inc_encoder.h"
#include "../periphs/uart.h"
#include <stdbool.h>
#include <stddef.h>

// Per-motor RS485 comms state, indexed by FaultMotorSlot. A read failure must
// persist for MOTOR_COMM_FAIL_DEBOUNCE consecutive replies before the motor is
// declared offline, so a lone dropped/garbled frame -- common on the half-duplex
// bus -- doesn't blip the RS485 fault bit. The streak is reset by the first good
// read. At 400 Hz each motor is read once per tick, so 4 ~= 10 ms.
#define MOTOR_COMM_FAIL_DEBOUNCE 4

static uint8_t motor_fail_count[3] = {0, 0, 0};        // consecutive failed reads
static bool    motor_offline[3]    = {false, false, false};  // debounced state
static uint8_t motor_meta[3]       = {0, 0, 0};        // last reported per-motor meta byte

// Map an RS485 motor address to its fault slot. Returns false for broadcast or
// any unrecognised address.
static bool addr_to_slot(uint8_t addr, FaultMotorSlot *slot) {
    switch (addr) {
        case MOTOR_COMMS_ADDR_TAKEUP:  *slot = FAULT_MOTOR_TAKEUP;  return true;
        case MOTOR_COMMS_ADDR_SUPPLY:  *slot = FAULT_MOTOR_SUPPLY;  return true;
        case MOTOR_COMMS_ADDR_CAPSTAN: *slot = FAULT_MOTOR_CAPSTAN; return true;
        default: return false;
    }
}

// Pull the one-byte meta-fault out of a reply payload. The reel-torque and
// fault-status replies are [cmd][meta]; a bare command echo (e.g. a capstan
// speed ack) carries no meta, so the motor is simply "online, nothing to report".
static uint8_t decode_motor_fault_byte(const uint8_t *rx_data, uint8_t rx_len) {
    if (rx_data == NULL || rx_len < 2) {
        return 0;
    }
    if (rx_data[0] == MOTOR_COMMS_CMD_REEL_TORQUE ||
        rx_data[0] == MOTOR_COMMS_CMD_FAULT_STATUS) {
        return rx_data[1];
    }
    return 0;
}

// Publish the RS485 fault bit: set whenever any motor is (debounced) offline.
// This is purely a comms-health bit -- actual motor/DRV faults are carried in the
// per-motor bytes, so they are intentionally NOT folded in here.
static void update_rs485_aggregate(void) {
    bool any_offline = false;
    for (int i = 0; i < 3; i++) {
        if (motor_offline[i]) {
            any_offline = true;
            break;
        }
    }
    if (any_offline) {
        command_center_set_ctrl_fault(CTRL_FAULT_RS485);
    } else {
        command_center_clear_ctrl_fault(CTRL_FAULT_RS485);
    }
}

void faults_init(void) {
    for (int i = 0; i < 3; i++) {
        motor_fail_count[i] = 0;
        motor_offline[i] = false;
        motor_meta[i] = 0;
    }
    command_center_clear_ctrl_fault(0xFF);
    command_center_set_motor_fault(FAULT_MOTOR_TAKEUP, 0);
    command_center_set_motor_fault(FAULT_MOTOR_SUPPLY, 0);
    command_center_set_motor_fault(FAULT_MOTOR_CAPSTAN, 0);
}

void faults_note_motor_reply(uint8_t motor_addr, RXError err,
                             const uint8_t *rx_data, uint8_t rx_len) {
    FaultMotorSlot slot;
    if (!addr_to_slot(motor_addr, &slot)) {
        return;  // broadcast or unknown address: nothing to attribute
    }

    if (err != RX_ERR_OK) {
        // Failed read: grow the failure streak and only declare the motor offline
        // once the debounce threshold is crossed. Hold the last-known motor byte
        // meanwhile -- a single dropped frame shouldn't disturb the reported state.
        if (motor_fail_count[slot] < MOTOR_COMM_FAIL_DEBOUNCE) {
            motor_fail_count[slot]++;
        }
        if (motor_fail_count[slot] >= MOTOR_COMM_FAIL_DEBOUNCE) {
            motor_offline[slot] = true;
        }
    } else {
        // Good read: clear the failure streak and store the reported meta byte.
        motor_fail_count[slot] = 0;
        motor_offline[slot] = false;
        uint8_t fb = decode_motor_fault_byte(rx_data, rx_len);
        motor_meta[slot] = fb;
        command_center_set_motor_fault(slot, fb);
    }

    update_rs485_aggregate();
}

bool faults_disarm_required(void) {
#if DISARM_ON_FAULT
    // Disarm on any non-zero fault byte: the controller meta byte (tension /
    // roller / RS485) or any per-motor reported fault byte. This is exactly the
    // set of bytes reported to uha_user in UCOMM_S_SEND_FAULTS.
    if (command_center_get_ctrl_fault() != 0) {
        return true;
    }
    for (int i = 0; i < 3; i++) {
        if (motor_meta[i] != 0) {
            return true;
        }
    }
#endif
    return false;
}

// --- uha_mother's own sensor health -----------------------------------------

void faults_check_health(void) {
    static bool prev_takeup = false;
    static bool prev_supply = false;

    // Tension-arm error state. We inspect only the frame the 200 Hz control loop
    // already cached over SPI -- no SPI access from here, since a second async
    // read corrupts the shared SERCOM3 bus the control loop depends on. Flags a
    // stuck line (disconnected/unpowered arm) or a parity-failed frame.
    bool takeup_bad = tension_takeup_error();
    bool supply_bad = tension_supply_error();

    if (takeup_bad) {
        command_center_set_ctrl_fault(CTRL_FAULT_TENSION_TAKEUP);
    } else {
        command_center_clear_ctrl_fault(CTRL_FAULT_TENSION_TAKEUP);
    }
    if (supply_bad) {
        command_center_set_ctrl_fault(CTRL_FAULT_TENSION_SUPPLY);
    } else {
        command_center_clear_ctrl_fault(CTRL_FAULT_TENSION_SUPPLY);
    }

    // Roller-encoder health: a robust stall detector needs the movement layer to
    // say "tape should be moving", which isn't plumbed here yet. Keep the bit
    // wired and clear until that detector lands (see plan: wire actual detectors
    // later) so we don't emit false faults.
    command_center_clear_ctrl_fault(CTRL_FAULT_ROLLER);

    // Print only on transitions so the main loop doesn't spam UART.
    if (takeup_bad && !prev_takeup)      uart_println("!! CTRL FAULT: takeup tension error state !!");
    else if (!takeup_bad && prev_takeup) uart_println("CTRL FAULT cleared: takeup tension");
    if (supply_bad && !prev_supply)      uart_println("!! CTRL FAULT: supply tension error state !!");
    else if (!supply_bad && prev_supply) uart_println("CTRL FAULT cleared: supply tension");
    prev_takeup = takeup_bad;
    prev_supply = supply_bad;
}

void faults_print_ctrl(void) {
    uint8_t c = command_center_get_ctrl_fault();
    uart_print("CTRL_FAULT: 0b");
    uart_println_int_base(c, 2);
    if (c & CTRL_FAULT_TENSION_TAKEUP) uart_println("  TENSION_TAKEUP: takeup arm error state");
    if (c & CTRL_FAULT_TENSION_SUPPLY) uart_println("  TENSION_SUPPLY: supply arm error state");
    if (c & CTRL_FAULT_ROLLER)         uart_println("  ROLLER: roller encoder fault");
    if (c & CTRL_FAULT_RS485)          uart_println("  RS485: a motor stopped responding on RS485");
}
