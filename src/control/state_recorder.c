#include "state_recorder.h"
#include "../periphs/timer.h"
#include "../periphs/gpio.h"
//#include "../periphs/uart.h"
#include "../drivers/board.h"
#include "controller.h"


static volatile bool flag = false;

static void set_flag() {
    //gpio_toggle_pin(DEBUG_PIN);
    flag = true;
}

void state_recorder_schedule() {
    timer_schedule(TIMER_ID_STATE_RECORDER, TIMER_SAMPLE_RATE_STATE_RECORDER, TIMER_PRIORITY_STATE_RECORDER, set_flag);
}

bool state_recorder_should_transmit() {
    return flag;
}

void state_recorder_transmit() {
    //gpio_set_pin(DEBUG_PIN);
    controller_send_state_uart();
    //gpio_clear_pin(DEBUG_PIN);
    flag = false;
}

