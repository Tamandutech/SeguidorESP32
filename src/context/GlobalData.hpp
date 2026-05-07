#pragma once

#include <vector>

#include "data_types.hpp"

/// Shared robot configuration and map (CLI writes when idle; control reads).
struct GlobalData {
  std::vector<MapPoint> mapData{};
  ParametersConfig      parametersConfig{};
};

inline GlobalData globalData{};
