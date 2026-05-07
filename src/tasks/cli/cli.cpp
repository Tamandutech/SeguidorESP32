#include "cli.hpp"

#include <cctype>
#include <cstring>
#include <vector>

#include "esp_log.h"

#include "tasks/cli/commands/dispatch.hpp"
#include "tasks/cli/wire_protocol.hpp"
#include "tasks/StateMachineTask.hpp"

namespace {
const char     *TAG   = "cli";
StateMachineTask *g_cliSm = nullptr;
} // namespace

StateMachineTask *cli_active_state_machine() { return g_cliSm; }

int cli(char *command, StateMachineTask *stateMachine) {
  if(command == nullptr || stateMachine == nullptr) {
    return CLI_ERROR_EMPTY_COMMAND;
  }

  while(*command != '\0' && isspace(static_cast<unsigned char>(*command))) {
    command++;
  }
  if(*command == '\0') {
    ESP_LOGI(TAG, "Empty command");
    return CLI_ERROR_EMPTY_COMMAND;
  }

  size_t n = strlen(command);
  while(n > 0 && isspace(static_cast<unsigned char>(command[n - 1]))) {
    command[--n] = '\0';
  }

  std::vector<std::pair<size_t, size_t>> segs;
  wire::splitTopLevel(command, strlen(command), ';', segs);
  if(segs.empty()) {
    return CLI_ERROR_EMPTY_COMMAND;
  }
  if(segs.size() > 48) {
    ESP_LOGW(TAG, "Too many wire segments");
    return CLI_ERROR_TOO_MANY_ARGS;
  }

  std::vector<wire::Command> cmds;
  cmds.reserve(segs.size());
  for(const auto &pr : segs) {
    wire::Command wc{};
    if(!wire::parseSegment(command + pr.first, pr.second - pr.first, wc)) {
      ESP_LOGW(TAG, "Bad wire segment");
      return CLI_ERROR_COMMAND_NOT_FOUND;
    }
    cmds.push_back(wc);
  }

  g_cliSm = stateMachine;
  const int ret = cli_dispatch::processWireCommands(cmds);
  g_cliSm = nullptr;
  return ret;
}
