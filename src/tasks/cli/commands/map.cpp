#include "map.hpp"

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "esp_err.h"
#include "esp_log.h"

#include "context/GlobalData.hpp"
#include "data_types.hpp"
#include "storage/storage.hpp"
#include "tasks/cli/cli.hpp"

namespace cli_map {

namespace {
const char *TAG = "cli_map";
} // namespace

bool parseMapAddBodyFields(const WireCommand &w, int argStart,
                           int32_t *vacuumPWM, int32_t *encMedia,
                           int *trackStatus, int32_t *offset) {
  if(w.argc < argStart + 4) {
    return false;
  }
  *vacuumPWM   = static_cast<int32_t>(atoi(w.argv[argStart]));
  *encMedia    = static_cast<int32_t>(atoi(w.argv[argStart + 1]));
  *trackStatus = atoi(w.argv[argStart + 2]);
  *offset      = static_cast<int32_t>(atoi(w.argv[argStart + 3]));
  return true;
}

int wireMapAddBody(const WireCommand &w, bool sortAfter) {
  int32_t vacuumPWM, encMedia, offset;
  int     trackStatus = 0;
  if(!parseMapAddBodyFields(w, 3, &vacuumPWM, &encMedia, &trackStatus,
                            &offset)) {
    ESP_LOGW(TAG, "map_add body: need 4 fields after idx");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  MapPoint point;
  point.encoderMilimeters = encMedia;
  point.baseMotorPWM      = offset;
  point.baseVacuumPWM     = vacuumPWM;
  point.markType          = static_cast<MapPoint::MarkType>(trackStatus);
  globalData.mapData.push_back(point);
  if(sortAfter) {
    std::sort(globalData.mapData.begin(), globalData.mapData.end(),
              [](const MapPoint &a, const MapPoint &b) {
                return a.encoderMilimeters < b.encoderMilimeters;
              });
    wire::emitSingleResponse("map_add", {"ok"});
  }
  return CLI_SUCCESS;
}

int wireMapClear() {
  globalData.mapData.clear();
  wire::emitSingleResponse("map_clear", {"ok"});
  return CLI_SUCCESS;
}

int wireMapClearStorage() {
  Storage              *storage = Storage::getInstance();
  std::vector<MapPoint> emptyMap;
  esp_err_t             ret = storage->write_vector(emptyMap, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "map_clear_storage failed (%s)", esp_err_to_name(ret));
    wire::emitSingleResponse("map_clear_storage",
                             {"error", "Failed to clear Flash"});
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  wire::emitSingleResponse("map_clear_storage", {"ok"});
  return CLI_SUCCESS;
}

int wireMapSave() {
  Storage  *storage = Storage::getInstance();
  esp_err_t ret = storage->write_vector(globalData.mapData, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "map_save failed (%s)", esp_err_to_name(ret));
    wire::emitSingleResponse("map_save", {"error", "Failed to save to Flash"});
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  wire::emitSingleResponse("map_save", {"ok"});
  return CLI_SUCCESS;
}

int wireMapGet() {
  std::vector<std::string> bodies;
  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    char            vacuum[16], enc[16], mark[16], motor[16];
    snprintf(vacuum, sizeof(vacuum), "%ld",
             static_cast<long>(point.baseVacuumPWM));
    snprintf(enc, sizeof(enc), "%ld",
             static_cast<long>(point.encoderMilimeters));
    snprintf(mark, sizeof(mark), "%d", static_cast<int>(point.markType));
    snprintf(motor, sizeof(motor), "%ld",
             static_cast<long>(point.baseMotorPWM));
    char   piece[wire::kWireLineBufferSize];
    size_t pp = 0;
    if(!wire::appendListBody(piece, sizeof(piece), pp, "map_get", 'b', 's',
                             static_cast<int>(i + 1),
                             {vacuum, enc, mark, motor})) {
      return CLI_ERROR_COMMAND_NOT_FOUND;
    }
    bodies.emplace_back(piece);
  }
  wire::emitListFromBodySegments("map_get", bodies);
  return CLI_SUCCESS;
}

} // namespace cli_map
