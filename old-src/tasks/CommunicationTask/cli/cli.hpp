#ifndef CLI_HPP
#define CLI_HPP

#include "esp_log.h"
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>
#include <vector>

#include "../lib/WireProtocol.hpp"
#include "context/GlobalData.hpp"
#include "context/RobotEnv.hpp"
#include "context/RobotStateMachine.hpp"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "storage/storage.hpp"

// CLI return codes
#define CLI_SUCCESS                 0
#define CLI_ERROR_EMPTY_COMMAND     1
#define CLI_ERROR_COMMAND_NOT_FOUND 2
#define CLI_ERROR_TOO_MANY_ARGS     3

// Storage files
#define MAP_STORAGE_FILE       "map_data.dat"
// FAT is 8.3 only (CONFIG_FATFS_LFN_NONE): basename must be <= 8 chars.
#define PARAMETERS_CONFIG_FILE "params.dat"

// Parse error codes for className.parameterName format
enum class ParseError {
  SUCCESS = 0,
  EMPTY_STRING,
  NO_DOT,
  MULTIPLE_DOTS,
  EMPTY_CLASS_NAME,
  EMPTY_PARAMETER_NAME,
  INVALID_FORMAT
};

// Structure to hold parsed className and parameterName
struct ParsedReference {
  char className[64];
  char parameterName[64];
};

// Parse a string in the format <className>.<parameterName>
// Returns ParseError enum indicating success or specific error
ParseError parseClassNameParameter(const char *input, ParsedReference &result) {
  // Check for empty string
  if(input == nullptr || input[0] == '\0') {
    return ParseError::EMPTY_STRING;
  }

  // Find the dot position
  const char *dotPos = strchr(input, '.');

  // Check if dot exists
  if(dotPos == nullptr) {
    return ParseError::NO_DOT;
  }

  // Check for multiple dots
  if(strchr(dotPos + 1, '.') != nullptr) {
    return ParseError::MULTIPLE_DOTS;
  }

  // Calculate lengths
  size_t classNameLen     = dotPos - input;
  size_t parameterNameLen = strlen(dotPos + 1);

  // Check if className is empty
  if(classNameLen == 0) {
    return ParseError::EMPTY_CLASS_NAME;
  }

  // Check if parameterName is empty
  if(parameterNameLen == 0) {
    return ParseError::EMPTY_PARAMETER_NAME;
  }

  // Check buffer sizes
  if(classNameLen >= sizeof(result.className) ||
     parameterNameLen >= sizeof(result.parameterName)) {
    return ParseError::INVALID_FORMAT;
  }

  // Copy className (before dot)
  strncpy(result.className, input, classNameLen);
  result.className[classNameLen] = '\0';

  // Copy parameterName (after dot)
  strncpy(result.parameterName, dotPos + 1, parameterNameLen);
  result.parameterName[parameterNameLen] = '\0';

  return ParseError::SUCCESS;
}

// Parse a float for param_set; supports optional leading '!' for negation (same
// convention as other parameters).
static bool parseCliFloat(const char *value, float *out) {
  if(value == nullptr || out == nullptr) {
    return false;
  }
  bool        isNegative  = (value[0] == '!');
  const char *actualValue = isNegative ? value + 1 : value;
  if(actualValue[0] == '\0') {
    return false;
  }
  char *end = nullptr;
  errno     = 0;
  float v   = strtof(actualValue, &end);
  if(end == actualValue || *end != '\0' || errno == ERANGE) {
    return false;
  }
  *out = isNegative ? -v : v;
  return true;
}

// Helper function to get parameter value as string
bool getParameterValue(const char *className, const char *parameterName,
                       char *valueBuffer, size_t bufferSize) {
  // State parameters
  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "runOnMappingMode") == 0) {
      snprintf(valueBuffer, bufferSize, "%d",
               globalData.parametersConfig.runOnMappingMode ? 1 : 0);
      return true;
    }
  }

  // Vacuum parameters
  if(strcmp(className, "Vacuum") == 0) {
    if(strcmp(parameterName, "speed") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               (long)globalData.parametersConfig.vacuumPWM);
      return true;
    }
  }

  if(strcmp(className, "Calibration") == 0) {
    if(strcmp(parameterName, "hardcodedCalibration") == 0) {
      snprintf(valueBuffer, bufferSize, "%d",
               globalData.parametersConfig.hardcodedCalibration ? 1 : 0);
      return true;
    }
  }

  if(strcmp(className, "Mapping") == 0) {
    if(strcmp(parameterName, "mappingMotorPWM") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               (long)globalData.parametersConfig.mappingMotorPWM);
      return true;
    }
  }

  if(strcmp(className, "PID") == 0) {
    if(strcmp(parameterName, "kP") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               (double)globalData.parametersConfig.pidKp);
      return true;
    }
    if(strcmp(parameterName, "kI") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               (double)globalData.parametersConfig.pidKi);
      return true;
    }
    if(strcmp(parameterName, "kD") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               (double)globalData.parametersConfig.pidKd);
      return true;
    }
  }

  return false;
}

// Helper function to set parameter value
bool setParameterValue(const char *className, const char *parameterName,
                       const char *value) {
  // Handle negative values (prefixed with !)
  bool        isNegative  = (value[0] == '!');
  const char *actualValue = isNegative ? value + 1 : value;

  // State parameters
  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "runOnMappingMode") == 0) {
      int val = atoi(actualValue);
      if(val == 1)
        globalData.parametersConfig.runOnMappingMode = true;
      else {
        globalData.parametersConfig.runOnMappingMode = false;
      }
      return true;
    }
  }

  if(strcmp(className, "Vacuum") == 0) {
    if(strcmp(parameterName, "speed") == 0) {
      int val = atoi(actualValue);
      if(val < 0) val = 0;
      if(val > 100) val = 100;
      globalData.parametersConfig.vacuumPWM = static_cast<int32_t>(val);
      return true;
    }
  }

  if(strcmp(className, "Calibration") == 0) {
    if(strcmp(parameterName, "hardcodedCalibration") == 0) {
      int val                                          = atoi(actualValue);
      globalData.parametersConfig.hardcodedCalibration = (val == 1);
      return true;
    }
  }

  if(strcmp(className, "Mapping") == 0) {
    if(strcmp(parameterName, "mappingMotorPWM") == 0) {
      int val = atoi(actualValue);
      if(val < 0) {
        val = 0;
      }
      if(val > RobotEnv::MAX_MOTOR_PWM) {
        val = RobotEnv::MAX_MOTOR_PWM;
      }
      globalData.parametersConfig.mappingMotorPWM = static_cast<int32_t>(val);
      return true;
    }
  }

  if(strcmp(className, "PID") == 0) {
    float v;
    if(!parseCliFloat(value, &v)) {
      return false;
    }
    if(strcmp(parameterName, "kP") == 0) {
      globalData.parametersConfig.pidKp = v;
      return true;
    }
    if(strcmp(parameterName, "kI") == 0) {
      globalData.parametersConfig.pidKi = v;
      return true;
    }
    if(strcmp(parameterName, "kD") == 0) {
      globalData.parametersConfig.pidKd = v;
      return true;
    }
  }

  return false;
}

using WireCommand = wire::Command;

// --- Command implementations (wire in / wire out) ------------------------

static int wireParamList() {
  struct Row {
    const char *nameCol;
    char        valueBuf[64];
  };
  std::vector<Row> rows;
  char             v[64];

  if(getParameterValue("State", "runOnMappingMode", v, sizeof(v))) {
    rows.push_back({"State.runOnMappingMode", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Vacuum", "speed", v, sizeof(v))) {
    rows.push_back({"Vacuum.speed", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Calibration", "hardcodedCalibration", v, sizeof(v))) {
    rows.push_back({"Calibration.hardcodedCalibration", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("Mapping", "mappingMotorPWM", v, sizeof(v))) {
    rows.push_back({"Mapping.mappingMotorPWM", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("PID", "kP", v, sizeof(v))) {
    rows.push_back({"PID.kP", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("PID", "kI", v, sizeof(v))) {
    rows.push_back({"PID.kI", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }
  if(getParameterValue("PID", "kD", v, sizeof(v))) {
    rows.push_back({"PID.kD", {}});
    strncpy(rows.back().valueBuf, v, sizeof(rows.back().valueBuf) - 1);
  }

  std::vector<std::string> rowWire;
  rowWire.reserve(rows.size());
  for(const Row &r : rows) {
    char piece[MESSAGE_LOG_MESSAGE_SIZE];
    size_t pp = 0;
    char idxs[16];
    snprintf(idxs, sizeof(idxs), "%d", static_cast<int>(rowWire.size()) + 1);
    if(!wire::appendListBody(piece, sizeof(piece), pp, "param_list", 'b', 's',
                      static_cast<int>(rowWire.size()) + 1,
                      {r.nameCol, r.valueBuf})) {
      return CLI_ERROR_COMMAND_NOT_FOUND;
    }
    rowWire.emplace_back(piece);
  }
  wire::emitListFromBodySegments("param_list", rowWire);
  return CLI_SUCCESS;
}

static int wireParamGet(const WireCommand &w) {
  if(w.argc < 3) {
    ESP_LOGW("CLI", "param_get(s,r,ref) missing args\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  ParsedReference ref;
  ParseError pe = parseClassNameParameter(w.argv[2], ref);
  if(pe != ParseError::SUCCESS) {
    ESP_LOGW("CLI", "param_get: bad ref\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  char value[64];
  if(!getParameterValue(ref.className, ref.parameterName, value,
                        sizeof(value))) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  wire::emitSingleResponse("param_get", {value});
  return CLI_SUCCESS;
}

static int paramSetRamOnly(const char *refWire, const char *valueWire) {
  ParsedReference ref;
  ParseError      pe = parseClassNameParameter(refWire, ref);
  if(pe != ParseError::SUCCESS) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  if(!setParameterValue(ref.className, ref.parameterName, valueWire)) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return CLI_SUCCESS;
}

static bool paramSetPersistWireError() {
  Storage *storage = Storage::getInstance();
  if(!storage->is_mounted()) {
    ESP_LOGW("CLI", "param_set: storage not mounted");
    wire::emitSingleResponse("param_set", {"error", "storage not mounted"});
    return false;
  }
  if(storage->write(globalData.parametersConfig, PARAMETERS_CONFIG_FILE) !=
     ESP_OK) {
    wire::emitSingleResponse("param_set", {"error", "failed to save parameters"});
    return false;
  }
  return true;
}

static int wireParamSetSingle(const WireCommand &w) {
  if(w.argc < 4) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  if(paramSetRamOnly(w.argv[2], w.argv[3]) != CLI_SUCCESS) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  if(!paramSetPersistWireError()) {
    return CLI_SUCCESS;
  }
  wire::emitSingleResponse("param_set", {"ok"});
  return CLI_SUCCESS;
}

static int wireParamSetBodyRamOnly(const WireCommand &w) {
  if(w.argc < 5) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return paramSetRamOnly(w.argv[3], w.argv[4]);
}

static int wireParamSetLoneBody(const WireCommand &w) {
  if(w.argc < 5) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  if(paramSetRamOnly(w.argv[3], w.argv[4]) != CLI_SUCCESS) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  if(!paramSetPersistWireError()) {
    return CLI_SUCCESS;
  }
  wire::emitSingleResponse("param_set", {"ok"});
  return CLI_SUCCESS;
}

static int wireMapClear() {
  globalData.mapData.clear();
  wire::emitSingleResponse("map_clear", {"ok"});
  return CLI_SUCCESS;
}

static int wireMapClearStorage() {
  Storage              *storage = Storage::getInstance();
  std::vector<MapPoint> emptyMap;
  esp_err_t             ret = storage->write_vector(emptyMap, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE("CLI", "map_clear_storage failed (%s)", esp_err_to_name(ret));
    wire::emitSingleResponse("map_clear_storage", {"error", "Failed to clear Flash"});
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  wire::emitSingleResponse("map_clear_storage", {"ok"});
  return CLI_SUCCESS;
}

static bool parseMapAddBodyFields(const WireCommand &w, int argStart,
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

static int wireMapAddBody(const WireCommand &w, bool sortAfter) {
  int32_t vacuumPWM, encMedia, offset;
  int     trackStatus = 0;
  if(!parseMapAddBodyFields(w, 3, &vacuumPWM, &encMedia, &trackStatus,
                             &offset)) {
    ESP_LOGW("CLI", "map_add body: need 4 fields after idx\n");
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

static int wireMapSave() {
  Storage  *storage = Storage::getInstance();
  esp_err_t ret = storage->write_vector(globalData.mapData, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE("CLI", "map_save failed (%s)", esp_err_to_name(ret));
    wire::emitSingleResponse("map_save", {"error", "Failed to save to Flash"});
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  wire::emitSingleResponse("map_save", {"ok"});
  return CLI_SUCCESS;
}

static int wireMapGet() {
  std::vector<std::string> bodies;
  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    char            vacuum[16], enc[16], mark[16], motor[16], idxs[16];
    snprintf(idxs, sizeof(idxs), "%zu", i + 1);
    snprintf(vacuum, sizeof(vacuum), "%ld",
             static_cast<long>(point.baseVacuumPWM));
    snprintf(enc, sizeof(enc), "%ld",
             static_cast<long>(point.encoderMilimeters));
    snprintf(mark, sizeof(mark), "%d", static_cast<int>(point.markType));
    snprintf(motor, sizeof(motor), "%ld",
             static_cast<long>(point.baseMotorPWM));
    char piece[MESSAGE_LOG_MESSAGE_SIZE];
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

static int wirePause() {
  RobotStateMachine::toIdle(globalData.motorDriver, globalData.vacuumDriver);
  wire::emitSingleResponse("pause", {"ok"});
  return CLI_SUCCESS;
}

static int wireResume() {
  if(globalData.parametersConfig.runOnMappingMode) {
    RobotStateMachine::toMapping(globalData.encoderLeftDriver,
                                 globalData.encoderRightDriver,
                                 globalData.vacuumDriver);
  } else {
    RobotStateMachine::toRunning(globalData.encoderLeftDriver,
                                 globalData.encoderRightDriver,
                                 globalData.vacuumDriver);
  }
  wire::emitSingleResponse("resume", {"ok"});
  return CLI_SUCCESS;
}

// Initialize ADC for battery voltage reading (static handle)
static adc_oneshot_unit_handle_t getBatteryAdcHandle() {
  static adc_oneshot_unit_handle_t adc2_handle = nullptr;
  static bool                      initialized = false;

  if(!initialized) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id  = ADC_UNIT_2,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc2_handle);
    if(ret != ESP_OK) {
      ESP_LOGE("CLI", "Failed to initialize ADC2 for battery voltage");
      return nullptr;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config);
    if(ret != ESP_OK) {
      ESP_LOGE("CLI", "Failed to configure ADC2 channel for battery voltage");
      adc_oneshot_del_unit(adc2_handle);
      adc2_handle = nullptr;
      return nullptr;
    }

    initialized = true;
  }

  return adc2_handle;
}

static int wireBatVoltage(void) {
  adc_oneshot_unit_handle_t adc_handle = getBatteryAdcHandle();
  char                        mv[16];
  if(adc_handle == nullptr) {
    snprintf(mv, sizeof(mv), "%d", 0);
    wire::emitSingleResponse("bat_voltage", {mv});
    return CLI_SUCCESS;
  }

  int       adc_raw = 0;
  esp_err_t ret     = adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_raw);

  if(ret != ESP_OK) {
    ESP_LOGE("CLI", "Failed to read battery voltage ADC: %s",
             esp_err_to_name(ret));
    snprintf(mv, sizeof(mv), "%d", 0);
    wire::emitSingleResponse("bat_voltage", {mv});
    return CLI_SUCCESS;
  }

  uint32_t voltage_mv = (static_cast<uint32_t>(adc_raw) * 3300) / 4095;
  snprintf(mv, sizeof(mv), "%lu", static_cast<unsigned long>(voltage_mv));
  wire::emitSingleResponse("bat_voltage", {mv});
  return CLI_SUCCESS;
}

// --- Dispatch: hardcoded name → handler maps (case-insensitive keys) --------

typedef int (*WireSingleRequestFn)(const WireCommand &w);

static int wh_param_list(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireParamList();
}

static int wh_param_get(const WireCommand &w) {
  if(w.argc < 3) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireParamGet(w);
}

static int wh_param_set(const WireCommand &w) {
  if(w.argc < 4) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireParamSetSingle(w);
}

static int wh_map_clear(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireMapClear();
}

static int wh_map_clear_storage(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireMapClearStorage();
}

static int wh_map_save(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireMapSave();
}

static int wh_map_get(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireMapGet();
}

static int wh_pause(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wirePause();
}

static int wh_resume(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireResume();
}

static int wh_bat_voltage(const WireCommand &w) {
  if(w.argc != 2) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return wireBatVoltage();
}

static const std::unordered_map<std::string, WireSingleRequestFn> &
getWireSingleRequestMap() {
  static const std::unordered_map<std::string, WireSingleRequestFn> m = {
      {"param_list",        wh_param_list        },
      {"param_get",         wh_param_get         },
      {"param_set",         wh_param_set         },
      {"map_clear",         wh_map_clear         },
      {"map_clear_storage", wh_map_clear_storage },
      {"map_save",          wh_map_save          },
      {"map_get",           wh_map_get           },
      {"pause",             wh_pause             },
      {"resume",            wh_resume            },
      {"bat_voltage",       wh_bat_voltage       },
  };
  return m;
}

typedef int (*WireLoneListBodyFn)(const WireCommand &w);

static int whLone_map_add(const WireCommand &w) {
  return wireMapAddBody(w, true);
}

static int whLone_param_set(const WireCommand &w) {
  return wireParamSetLoneBody(w);
}

static const std::unordered_map<std::string, WireLoneListBodyFn> &
getWireLoneListBodyMap() {
  static const std::unordered_map<std::string, WireLoneListBodyFn> m = {
      {"map_add",   whLone_map_add   },
      {"param_set", whLone_param_set },
  };
  return m;
}

typedef int (*WireListHeaderBatchFn)(std::vector<WireCommand> &cmds,
                                     size_t headerIdx, int C, int j);

static int whBatch_map_add(std::vector<WireCommand> &cmds, size_t headerIdx,
                           int C, int j) {
  for(int k = 1; k <= C; k++) {
    int r = wireMapAddBody(cmds[headerIdx + static_cast<size_t>(k)], false);
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
    int r = wireParamSetBodyRamOnly(
        cmds[headerIdx + static_cast<size_t>(k)]);
    if(r != CLI_SUCCESS) {
      return r;
    }
  }
  if(!paramSetPersistWireError()) {
    return CLI_SUCCESS;
  }
  wire::emitBatchAck("param_set", j);
  return CLI_SUCCESS;
}

static const std::unordered_map<std::string, WireListHeaderBatchFn> &
getWireListHeaderBatchMap() {
  static const std::unordered_map<std::string, WireListHeaderBatchFn> m = {
      {"map_add",   whBatch_map_add   },
      {"param_set", whBatch_param_set },
  };
  return m;
}

static int dispatchWireSingleRequest(const WireCommand &w) {
  if(w.mode != 's' || w.role != 'r') {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  const auto &m = getWireSingleRequestMap();
  auto        it = m.find(wire::commandKeyLower(w.name));
  if(it == m.end()) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return it->second(w);
}

static int processWireCommands(std::vector<WireCommand> &cmds) {
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
        if(bk.mode != 'b' || bk.role != 'r' ||
           !wire::nameEq(bk.name, w.name)) {
          return CLI_ERROR_COMMAND_NOT_FOUND;
        }
      }
      const auto &hm = getWireListHeaderBatchMap();
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
      const auto &bm = getWireLoneListBodyMap();
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

int cli(char *command) {
  if(command == nullptr) {
    return CLI_ERROR_EMPTY_COMMAND;
  }
  while(*command != '\0' &&
        isspace(static_cast<unsigned char>(*command))) {
    command++;
  }
  if(*command == '\0') {
    ESP_LOGI("CLI", "Empty command\n");
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
    ESP_LOGW("CLI", "Too many wire segments\n");
    return CLI_ERROR_TOO_MANY_ARGS;
  }

  std::vector<WireCommand> cmds;
  cmds.reserve(segs.size());
  for(const auto &pr : segs) {
    WireCommand wc{};
    if(!wire::parseSegment(command + pr.first, pr.second - pr.first, wc)) {
      ESP_LOGW("CLI", "Bad wire segment\n");
      return CLI_ERROR_COMMAND_NOT_FOUND;
    }
    cmds.push_back(wc);
  }

  return processWireCommands(cmds);
}

#endif // CLI_HPP