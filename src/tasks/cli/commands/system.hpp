#pragma once

#include "tasks/cli/wire_protocol.hpp"

namespace cli_system {

using WireCommand = wire::Command;

int wirePause();
int wireResume();
int wireBatVoltage();

} // namespace cli_system
