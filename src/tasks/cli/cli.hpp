#pragma once

#include <stdint.h>

class StateMachineTask;

#define CLI_SUCCESS                 0
#define CLI_ERROR_EMPTY_COMMAND     1
#define CLI_ERROR_COMMAND_NOT_FOUND 2
#define CLI_ERROR_TOO_MANY_ARGS     3

/// Active only during \c cli execution (BLE UART command handling thread).
StateMachineTask *cli_active_state_machine();

/// Wire-protocol CLI (`param_list(...)`, `map_get(...)`, …); same grammar as
/// legacy project.
int cli(char *command, StateMachineTask *stateMachine);
