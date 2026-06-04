#pragma once

// Master Commands
#define UCOMM_M_STOP              (0x00)
#define UCOMM_M_PLAY              (0x01)
#define UCOMM_M_FAST_FORWARD      (0x02)
#define UCOMM_M_REWIND            (0x03)
#define UCOMM_M_SPOOL             (0x04)
#define UCOMM_M_SET_ZERO          (0x05)
// 0x06 reserved (was UCOMM_M_GOTO_ZERO; use UCOMM_M_GOTO_LOC with 0.0f)
#define UCOMM_M_GOTO_LOC          (0x07)
#define UCOMM_M_SET_CAPSTAN_SPEED (0x08)
#define UCOMM_M_SET_SPOOL_SPEED   (0x09)
#define UCOMM_M_SET_AUTO_PLAY     (0x0A)
#define UCOMM_M_CLEAR_AUTO_PLAY   (0x0B)
#define UCOMM_M_CLEAR_AUTO_LOOP   (0x0C)
#define UCOMM_M_SET_AUTO_LOOP     (0x0D)
#define UCOMM_M_GET_FAULTS        (0x0E)
#define UCOMM_M_GET_TAPE_POS      (0x0F)
#define UCOMM_M_GET_TAPE_SPEED    (0x10)
#define UCOMM_M_GET_TENSION_ARMS  (0x11)
#define UCOMM_M_GET_ODOMETER      (0x12)
#define UCOMM_M_GET_UI_STATE      (0x13)

// Slave Commands
// ACKs
#define UCOMM_S_ACK_STOP              (0x00)
#define UCOMM_S_ACK_PLAY              (0x01)
#define UCOMM_S_ACK_FAST_FORWARD      (0x02)
#define UCOMM_S_ACK_REWIND            (0x03)
#define UCOMM_S_ACK_SPOOL             (0x04)
#define UCOMM_S_ACK_SET_ZERO          (0x05)
// 0x06 reserved (was UCOMM_S_ACK_GOTO_ZERO)
#define UCOMM_S_ACK_GOTO_LOC          (0x07)
#define UCOMM_S_ACK_SET_CAPSTAN_SPEED (0x08)
#define UCOMM_S_ACK_SET_SPOOL_SPEED   (0x09)
#define UCOMM_S_ACK_SET_AUTO_PLAY     (0x0A)
#define UCOMM_S_ACK_CLEAR_AUTO_PLAY   (0x0B)
#define UCOMM_S_ACK_CLEAR_AUTO_LOOP   (0x0C)
#define UCOMM_S_ACK_SET_AUTO_LOOP     (0x0D)

// Return Data
#define UCOMM_S_SEND_FAULTS        (0x0E)
#define UCOMM_S_SEND_TAPE_POS      (0x0F)
#define UCOMM_S_SEND_TAPE_SPEED    (0x10)
#define UCOMM_S_SEND_TENSION_ARMS  (0x11)
#define UCOMM_S_SEND_ODOMETER      (0x12)
#define UCOMM_S_SEND_UI_STATE      (0x13)

// UI_STATE flag bits (byte 2 of UI_STATE response)
#define UCOMM_UI_FLAG_TRANSIENT    (1 << 0)

