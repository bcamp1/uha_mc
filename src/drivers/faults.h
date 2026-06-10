/*
 * faults.h
 *
 * uha_mother fault framework. Mirrors uha_mc/src/drivers/faults.c:
 *
 *   - Per-motor faults: each uha_mc slave replies to a reel-torque command with
 *     its one-byte meta-fault register. faults_note_motor_reply() harvests that
 *     reply (called from motors.c after every command) and deposits it as the
 *     motor's fault byte in command_center.
 *
 *   - Controller meta-fault (CTRL_FAULT_*): uha_mother's own fault byte, covering
 *     the two tension arms, the roller encoder, and a general motor-driver flag.
 *     faults_check_health() evaluates these from the main loop.
 *
 * The aggregate (fault_controller + the three motor bytes) is reported to
 * uha_user over RS422 when it sends UCOMM_M_GET_FAULTS (poll-only, like the way
 * uha_mc answers uha_mother's FAULT_STATUS query).
 */

#pragma once
#include <stdint.h>
#include "motor_comms.h"  // RXError, MOTOR_COMMS_ADDR_*

// Controller meta-fault register (the fault_controller byte sent to uha_user).
#define CTRL_FAULT_TENSION_TAKEUP  (1u << 0)  // takeup tension error state (stuck line / bad parity)
#define CTRL_FAULT_TENSION_SUPPLY  (1u << 1)  // supply tension error state (stuck line / bad parity)
#define CTRL_FAULT_ROLLER          (1u << 2)  // roller (inc) encoder fault
#define CTRL_FAULT_MOTOR_DRIVER    (1u << 3)  // a motor reported a fault or dropped off RS485

// Reset the controller meta-fault register and the internal motor shadow state.
void faults_init(void);

// Harvest a motor's reply. Call from motors.c after every command to a motor.
//   - err != RX_ERR_OK            -> motor offline: zero its byte, raise MOTOR_DRIVER
//   - reel-torque / fault-status reply ([cmd][meta]) -> store meta as its byte
//   - bare echo (e.g. capstan speed ack) -> motor online, no meta available
// rx_data/rx_len are the response payload from motor_comms_write_read (framing
// already stripped). rx_data may be NULL when only comms presence is known.
void faults_note_motor_reply(uint8_t motor_addr, RXError err,
                             const uint8_t *rx_data, uint8_t rx_len);

// Evaluate uha_mother's own sensors (tension arms, roller encoder) and update
// the CTRL_FAULT_* bits. Call from the main loop. Mirrors uha_mc
// faults_check_health().
void faults_check_health(void);

// Decode and print the controller meta-fault register over UART (debug parity
// with uha_mc faults_print_meta()).
void faults_print_ctrl(void);
