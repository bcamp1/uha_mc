#pragma once
#include <stdint.h>

typedef enum {
    CMD_STOP,
    CMD_PLAY,
    CMD_FAST_FORWARD,
    CMD_REWIND,
    CMD_SPOOL,
    CMD_SET_ZERO,
    CMD_SET_CAPSTAN_SPEED,
    CMD_GOTO_LOC,
    CMD_NONE,
} CommandCenterSimpleAction;

// Which per-motor fault byte to update. These select fault_takeup/supply/capstan,
// the bytes reported to uha_user in the UCOMM_S_SEND_FAULTS frame.
typedef enum {
    FAULT_MOTOR_TAKEUP = 0,
    FAULT_MOTOR_SUPPLY,
    FAULT_MOTOR_CAPSTAN,
} FaultMotorSlot;

void command_center_init();
CommandCenterSimpleAction command_center_get_action();

// Tape position (encoder units) requested by the most recent CMD_GOTO_LOC.
// Read after command_center_get_action() returns CMD_GOTO_LOC.
float command_center_get_goto_loc();

// --- Fault state (written by faults.c, reported over RS422) -----------------
// fault_controller is uha_mother's own meta-fault register (CTRL_FAULT_* bits in
// faults.h). The set/clear pair is a read-modify-write on one byte and is
// IRQ-safe (the movement ISR and the main loop both touch it).
void    command_center_set_ctrl_fault(uint8_t mask);    // OR-set  (|=)
void    command_center_clear_ctrl_fault(uint8_t mask);  // AND-NOT (&= ~)
uint8_t command_center_get_ctrl_fault(void);

// Store the meta-fault byte reported by a motor (single-byte atomic store).
void    command_center_set_motor_fault(FaultMotorSlot slot, uint8_t fault_byte);

