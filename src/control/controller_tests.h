#ifndef CONTROLLER_TESTS_H
#define CONTROLLER_TESTS_H
#include "controller.h"
#include <stdbool.h>

extern ControllerConfig controller_config_demo;
void controller_tests_run(ControllerConfig *config, bool send_logs, bool uart_toggle, bool start_on);

#endif
