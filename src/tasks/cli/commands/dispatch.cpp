#include "dispatch.hpp"

#include <algorithm>
#include <cstdlib>
#include <string>
#include <unordered_map>

#include <vector>

#include "context/GlobalData.hpp"
#include "data_types.hpp"
#include "tasks/cli/cli.hpp"
#include "tasks/cli/commands/map.hpp"
#include "tasks/cli/commands/param.hpp"
#include "tasks/cli/commands/system.hpp"
#include "tasks/cli/wire_protocol.hpp"

namespace cli_dispatch {

using WireCommand = wire::Command;

namespace {

typedef int (*WireSingleRequestFn)(const WireCommand &w);

static int wh_param_list(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_param::wireParamList();
}

static int wh_param_get(const WireCommand &w) {
  if(w.argc < 3) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_param::wireParamGet(w);
}

static int wh_param_set(const WireCommand &w) {
  if(w.argc < 4) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_param::wireParamSetSingle(w);
}

static int wh_map_clear(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_map::wireMapClear();
}

static int wh_map_clear_storage(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_map::wireMapClearStorage();
}

static int wh_map_save(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_map::wireMapSave();
}

static int wh_map_get(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_map::wireMapGet();
}

static int wh_pause(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_system::wirePause();
}

static int wh_resume(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_system::wireResume();
}

static int wh_bat_voltage(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return cli_system::wireBatVoltage();
}

static const std::unordered_map<std::string, WireSingleRequestFn> &
getWireSingleRequestMap() {
  static const std::unordered_map<std::string, WireSingleRequestFn> m = {
      {"param_list",        wh_param_list       },
      {"param_get",         wh_param_get        },
      {"param_set",         wh_param_set        },
      {"map_clear",         wh_map_clear        },
      {"map_clear_storage", wh_map_clear_storage},
      {"map_save",          wh_map_save         },
      {"map_get",           wh_map_get          },
      {"pause",             wh_pause            },
      {"resume",            wh_resume           },
      {"bat_voltage",       wh_bat_voltage      },
  };
  return m;
}

typedef int (*WireLoneListBodyFn)(const WireCommand &w);

static int whLone_map_add(const WireCommand &w) {
  return cli_map::wireMapAddBody(w, true);
}

static int whLone_param_set(const WireCommand &w) {
  return cli_param::wireParamSetLoneBody(w);
}

static const std::unordered_map<std::string, WireLoneListBodyFn> &
getWireLoneListBodyMap() {
  static const std::unordered_map<std::string, WireLoneListBodyFn> m = {
      {"map_add",   whLone_map_add  },
      {"param_set", whLone_param_set},
  };
  return m;
}

typedef int (*WireListHeaderBatchFn)(std::vector<WireCommand> &cmds,
                                     size_t headerIdx, int C, int j);

static int whBatch_map_add(std::vector<WireCommand> &cmds, size_t headerIdx,
                           int C, int j) {
  for(int k = 1; k <= C; k++) {
    int r = cli_map::wireMapAddBody(cmds[headerIdx + static_cast<size_t>(k)],
                                    false);
    if(r != CLI_SUCCESS) {
      return r;
    }
  }
  std::sort(globalData.mapData.begin(), globalData.mapData.end(),
            [](const MapPoint &a, const MapPoint &b) {
              return a.encoderMilimeters < b.encoderMilimeters;
            });
  wire::emitBatchAck("map_add", j);
  return CLI_SUCCESS;
}

static int whBatch_param_set(std::vector<WireCommand> &cmds, size_t headerIdx,
                             int C, int j) {
  for(int k = 1; k <= C; k++) {
    int r = cli_param::wireParamSetBodyRamOnly(
        cmds[headerIdx + static_cast<size_t>(k)]);
    if(r != CLI_SUCCESS) {
      return r;
    }
  }
  if(!cli_param::paramSetPersistWireError()) {
    return CLI_SUCCESS;
  }
  wire::emitBatchAck("param_set", j);
  return CLI_SUCCESS;
}

static const std::unordered_map<std::string, WireListHeaderBatchFn> &
getWireListHeaderBatchMap() {
  static const std::unordered_map<std::string, WireListHeaderBatchFn> m = {
      {"map_add",   whBatch_map_add  },
      {"param_set", whBatch_param_set},
  };
  return m;
}

static int dispatchWireSingleRequest(const WireCommand &w) {
  if(w.mode != 's' || w.role != 'r') {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  const auto &m  = getWireSingleRequestMap();
  auto        it = m.find(wire::commandKeyLower(w.name));
  if(it == m.end()) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return it->second(w);
}

} // namespace

int processWireCommands(std::vector<wire::Command> &cmds) {
  for(size_t i = 0; i < cmds.size();) {
    WireCommand &w = cmds[i];
    if(w.role != 'r') {
      i++;
      continue;
    }
    if(w.mode == 'h') {
      if(w.argc < 6) {
        return CLI_ERROR_COMMAND_NOT_FOUND;
      }
      int T = atoi(w.argv[2]);
      int C = atoi(w.argv[3]);
      int B = atoi(w.argv[4]);
      int j = atoi(w.argv[5]);
      (void)T;
      (void)B;
      if(C < 0 || i + 1 + static_cast<size_t>(C) > cmds.size()) {
        return CLI_ERROR_COMMAND_NOT_FOUND;
      }
      for(int k = 1; k <= C; k++) {
        WireCommand &bk = cmds[i + static_cast<size_t>(k)];
        if(bk.mode != 'b' || bk.role != 'r' || !wire::nameEq(bk.name, w.name)) {
          return CLI_ERROR_COMMAND_NOT_FOUND;
        }
      }
      const auto &hm  = getWireListHeaderBatchMap();
      auto        hit = hm.find(wire::commandKeyLower(w.name));
      if(hit == hm.end()) {
        return CLI_ERROR_COMMAND_NOT_FOUND;
      }
      int r = hit->second(cmds, i, C, j);
      if(r != CLI_SUCCESS) {
        return r;
      }
      i += 1 + static_cast<size_t>(C);
      continue;
    }
    if(w.mode == 'b') {
      const auto &bm  = getWireLoneListBodyMap();
      auto        bit = bm.find(wire::commandKeyLower(w.name));
      if(bit == bm.end()) {
        return CLI_ERROR_COMMAND_NOT_FOUND;
      }
      int r = bit->second(w);
      i++;
      if(r != CLI_SUCCESS) {
        return r;
      }
      continue;
    }
    if(w.mode == 's') {
      int r = dispatchWireSingleRequest(w);
      i++;
      if(r != CLI_SUCCESS) {
        return r;
      }
      continue;
    }
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return CLI_SUCCESS;
}

} // namespace cli_dispatch
