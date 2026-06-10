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

// Shadow of per-motor state, kept here so the aggregate CTRL_FAULT_MOTOR_DRIVER
// bit can be recomputed without reading back command_center. Indexed by
// FaultMotorSlot.
static uint8_t motor_fault[3] = {0, 0, 0};
static bool    motor_offline[3] = {false, false, false};

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

// Recompute and publish the aggregate motor-driver bit: set whenever any motor
// is offline or is reporting a non-zero meta-fault.
static void update_motor_driver_aggregate(void) {
    bool faulted = false;
    for (int i = 0; i < 3; i++) {
        if (motor_offline[i] || motor_fault[i] != 0) {
            faulted = true;
            break;
        }
    }
    if (faulted) {
        command_center_set_ctrl_fault(CTRL_FAULT_MOTOR_DRIVER);
    } else {
        command_center_clear_ctrl_fault(CTRL_FAULT_MOTOR_DRIVER);
    }
}

void faults_init(void) {
    for (int i = 0; i < 3; i++) {
        motor_fault[i] = 0;
        motor_offline[i] = false;
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
        // Motor did not answer / bad frame: treat as offline, ambiguous fault.
        motor_offline[slot] = true;
        motor_fault[slot] = 0;
        command_center_set_motor_fault(slot, 0);
    } else {
        motor_offline[slot] = false;
        uint8_t fb = decode_motor_fault_byte(rx_data, rx_len);
        motor_fault[slot] = fb;
        command_center_set_motor_fault(slot, fb);
    }

    update_motor_driver_aggregate();
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
    if (c & CTRL_FAULT_MOTOR_DRIVER)   uart_println("  MOTOR_DRIVER: a motor faulted or dropped off RS485");
}
