#include "state_recorder.h"
#include "../periphs/timer.h"
#include "../periphs/gpio.h"
//#include "../periphs/uart.h"
#include "controller.h"

#define STATE_RECORDER_SAMPLE_RATE  (50.0f)
#define STATE_RECORDER_TIMER_ID     (2)

#define DEBUG_PIN PIN_PA14
#define LED PIN_PA15

static volatile bool flag = false;

static void set_flag() {
    //gpio_toggle_pin(DEBUG_PIN);
    flag = true;
}

void state_recorder_schedule() {
    timer_schedule(STATE_RECORDER_TIMER_ID, STATE_RECORDER_SAMPLE_RATE, set_flag);
}

bool state_recorder_should_transmit() {
    return flag;
}

void state_recorder_transmit() {
    gpio_set_pin(DEBUG_PIN);
    controller_send_state_uart();
    gpio_clear_pin(DEBUG_PIN);
    flag = false;
}

