#include "command_center.h"
#include "ucomm_commands.h"
#include "user_comms.h"
#include "../periphs/gpio.h"
#include "../board.h"
#include "../control/data_collector.h"
#include <sam.h>
#include <stdint.h>
#include <stdbool.h>

// These variables are managed by us, sent to the user
static uint8_t fault_controller = 0;
static uint8_t fault_takeup = 0;
static uint8_t fault_supply = 0;
static uint8_t fault_capstan = 0;
static float odometer_distance = 0;
static float odometer_time = 0;

// These variables are managed by user, sent to us
volatile CommandCenterSimpleAction latest_action = CMD_NONE;
float latest_goto_loc = 0.0f;
bool is_auto_play = false;
bool is_auto_loop = false;
float auto_loop_loc1 = 0.0f;
float auto_loop_loc2 = 0.0f;

// RX message callback
static void rx_callback() {
    gpio_set_pin(PIN_DEBUG2);
    gpio_clear_pin(PIN_DEBUG2);
    CommsRxResult rx = comms_get_data();
    if (rx.err != RX_ERR_OK || rx.data_len < 1) {
        return;
    }

    uint8_t command = rx.data[0];
    uint8_t response[10];

    //comms_send_byte(command);
    //return;

    switch (command) {
        case UCOMM_M_STOP:
            comms_send_byte(UCOMM_S_ACK_STOP);
            latest_action = CMD_STOP;
            break;
        case UCOMM_M_PLAY:
            comms_send_byte(UCOMM_S_ACK_PLAY);
            latest_action = CMD_PLAY;
            break;
        case UCOMM_M_FAST_FORWARD:
            comms_send_byte(UCOMM_S_ACK_FAST_FORWARD);
            latest_action = CMD_FAST_FORWARD;
            break;
        case UCOMM_M_REWIND:
            comms_send_byte(UCOMM_S_ACK_REWIND);
            latest_action = CMD_REWIND;
            break;
        case UCOMM_M_SPOOL:
            comms_send_byte(UCOMM_S_ACK_SPOOL);
            latest_action = CMD_SPOOL;
            break;
        case UCOMM_M_SET_ZERO:
            comms_send_byte(UCOMM_S_ACK_SET_ZERO);
            latest_action = CMD_SET_ZERO;
            break;
        case UCOMM_M_GOTO_ZERO:
            comms_send_byte(UCOMM_S_ACK_GOTO_ZERO);
            break;
        case UCOMM_M_GOTO_LOC:
            comms_send_byte(UCOMM_S_ACK_GOTO_LOC);
            break;
        case UCOMM_M_SET_CAPSTAN_SPEED:
            comms_send_byte(UCOMM_S_ACK_SET_CAPSTAN_SPEED);
            latest_action = CMD_SET_CAPSTAN_SPEED;
            break;
        case UCOMM_M_SET_SPOOL_SPEED:
            comms_send_byte(UCOMM_S_ACK_SET_SPOOL_SPEED);
            break;
        case UCOMM_M_SET_AUTO_PLAY:
            comms_send_byte(UCOMM_S_ACK_SET_AUTO_PLAY);
            break;
        case UCOMM_M_CLEAR_AUTO_PLAY:
            comms_send_byte(UCOMM_S_ACK_CLEAR_AUTO_PLAY);
            break;
        case UCOMM_M_CLEAR_AUTO_LOOP:
            comms_send_byte(UCOMM_S_ACK_CLEAR_AUTO_LOOP);
            break;
        case UCOMM_M_SET_AUTO_LOOP:
            comms_send_byte(UCOMM_S_ACK_SET_AUTO_LOOP);
            break;
        case UCOMM_M_GET_FAULTS:
            response[0] = UCOMM_S_SEND_FAULTS;
            response[1] = fault_controller;
            response[2] = fault_takeup;
            response[3] = fault_supply;
            response[4] = fault_capstan;
            comms_send_bytes(response, 5);
            break;
        case UCOMM_M_GET_TAPE_POS: {
            float tape_pos = data_collector_get_tape_position();
            comms_send_float(UCOMM_S_SEND_TAPE_POS, tape_pos);
            break;
        }
        case UCOMM_M_GET_TAPE_SPEED: {
            float tape_speed = data_collector_get_tape_speed();
            comms_send_float(UCOMM_S_SEND_TAPE_SPEED, tape_speed);
            break;
        }   
        case UCOMM_M_GET_TENSION_ARMS: {
            float tension_takeup = data_collector_get_takeup_tension();
            float tension_supply = data_collector_get_supply_tension();
            if (tension_takeup < 0.0f) tension_takeup = 0.0f;
            if (tension_supply < 0.0f) tension_supply = 0.0f;
            uint8_t t = (uint8_t) (tension_takeup * 255.0f);
            uint8_t s = (uint8_t) (tension_supply * 255.0f);
            response[0] = UCOMM_S_SEND_TENSION_ARMS;
            response[1] = t;
            response[2] = s;
            comms_send_bytes(response, 3);
            break;
        }
        case UCOMM_M_GET_ODOMETER:
            comms_send_float2(UCOMM_S_SEND_ODOMETER, odometer_time, odometer_distance);
            break;
        default:
            break;
    }
}

CommandCenterSimpleAction command_center_get_action() {
    __disable_irq();
    CommandCenterSimpleAction action = latest_action;
    latest_action = CMD_NONE;
    __enable_irq();
    return action;
}

void command_center_init() {
    comms_init();
    comms_register_message_ready_cb(rx_callback);
}

