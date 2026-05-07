#pragma once

#include <vector>

#include "tasks/cli/wire_protocol.hpp"

namespace cli_dispatch {

int processWireCommands(std::vector<wire::Command> &cmds);

}
