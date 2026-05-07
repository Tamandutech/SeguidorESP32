#pragma once

#include <cstdint>

#include "tasks/cli/wire_protocol.hpp"

namespace cli_map {

using WireCommand = wire::Command;

bool parseMapAddBodyFields(const WireCommand &w, int argStart,
                           int32_t *vacuumPWM, int32_t *encMedia,
                           int *trackStatus, int32_t *offset);

int wireMapAddBody(const WireCommand &w, bool sortAfter);

int wireMapClear();
int wireMapClearStorage();
int wireMapSave();
int wireMapGet();

} // namespace cli_map
