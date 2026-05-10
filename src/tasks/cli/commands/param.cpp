#include "param.hpp"

#include <cctype>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "esp_log.h"

#include "context/GlobalData.hpp"
#include "env.hpp"
#include "storage/storage.hpp"
#include "tasks/cli/cli.hpp"

namespace cli_param {

namespace {
const char *TAG = "cli_param";
} // namespace

ParseError parseClassNameParameter(const char *input, ParsedReference &result) {
  if(input == nullptr || input[0] == '\0') {
    return ParseError::EMPTY_STRING;
  }

  const char *dotPos = strchr(input, '.');
  if(dotPos == nullptr) {
    return ParseError::NO_DOT;
  }
  if(strchr(dotPos + 1, '.') != nullptr) {
    return ParseError::MULTIPLE_DOTS;
  }

  size_t classNameLen     = static_cast<size_t>(dotPos - input);
  size_t parameterNameLen = strlen(dotPos + 1);

  if(classNameLen == 0) {
    return ParseError::EMPTY_CLASS_NAME;
  }
  if(parameterNameLen == 0) {
    return ParseError::EMPTY_PARAMETER_NAME;
  }

  if(classNameLen >= sizeof(result.className) ||
     parameterNameLen >= sizeof(result.parameterName)) {
    return ParseError::INVALID_FORMAT;
  }

  memcpy(result.className, input, classNameLen);
  result.className[classNameLen] = '\0';

  memcpy(result.parameterName, dotPos + 1, parameterNameLen);
  result.parameterName[parameterNameLen] = '\0';

  return ParseError::SUCCESS;
}

bool parseCliFloat(const char *value, float *out) {
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

bool getParameterValue(const char *className, const char *parameterName,
                       char *valueBuffer, size_t bufferSize) {
  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "runOnMappingMode") == 0) {
      snprintf(valueBuffer, bufferSize, "%d",
               globalData.parametersConfig.runOnMappingMode ? 1 : 0);
      return true;
    }
  }

  if(strcmp(className, "Vacuum") == 0) {
    if(strcmp(parameterName, "speed") == 0) {
      snprintf(valueBuffer, bufferSize, "%ld",
               static_cast<long>(globalData.parametersConfig.vacuumPWM));
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
               static_cast<long>(globalData.parametersConfig.mappingMotorPWM));
      return true;
    }
  }

  if(strcmp(className, "PID") == 0) {
    if(strcmp(parameterName, "kP") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               static_cast<double>(globalData.parametersConfig.pidKp));
      return true;
    }
    if(strcmp(parameterName, "kI") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               static_cast<double>(globalData.parametersConfig.pidKi));
      return true;
    }
    if(strcmp(parameterName, "kD") == 0) {
      snprintf(valueBuffer, bufferSize, "%.6f",
               static_cast<double>(globalData.parametersConfig.pidKd));
      return true;
    }
  }

  return false;
}

bool setParameterValue(const char *className, const char *parameterName,
                       const char *value) {
  bool        isNegative  = (value[0] == '!');
  const char *actualValue = isNegative ? value + 1 : value;

  if(strcmp(className, "State") == 0) {
    if(strcmp(parameterName, "runOnMappingMode") == 0) {
      int val                                      = atoi(actualValue);
      globalData.parametersConfig.runOnMappingMode = (val == 1);
      return true;
    }
  }

  if(strcmp(className, "Vacuum") == 0) {
    if(strcmp(parameterName, "speed") == 0) {
      int val = atoi(actualValue);
      if(val < 0) {
        val = 0;
      }
      if(val > 100) {
        val = 100;
      }
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
      if(val > MAX_MOTOR_PWM) {
        val = MAX_MOTOR_PWM;
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

bool paramSetPersistWireError() {
  Storage *storage = Storage::getInstance();
  if(!storage->is_mounted()) {
    ESP_LOGW(TAG, "param_set: storage not mounted");
    wire::emitSingleResponse("param_set", {"error", "storage not mounted"});
    return false;
  }
  if(storage->write(globalData.parametersConfig, PARAMETERS_STORAGE_FILE) !=
     ESP_OK) {
    wire::emitSingleResponse("param_set",
                             {"error", "failed to save parameters"});
    return false;
  }
  return true;
}

int paramSetRamOnly(const char *refWire, const char *valueWire) {
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

int wireParamList() {
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
    char   piece[wire::kWireLineBufferSize];
    size_t pp = 0;
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

int wireParamGet(const WireCommand &w) {
  if(w.argc < 3) {
    ESP_LOGW(TAG, "param_get(s,r,ref) missing args");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  ParsedReference ref;
  ParseError      pe = parseClassNameParameter(w.argv[2], ref);
  if(pe != ParseError::SUCCESS) {
    ESP_LOGW(TAG, "param_get: bad ref");
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

int wireParamSetSingle(const WireCommand &w) {
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

int wireParamSetBodyRamOnly(const WireCommand &w) {
  if(w.argc < 5) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  return paramSetRamOnly(w.argv[3], w.argv[4]);
}

int wireParamSetLoneBody(const WireCommand &w) {
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

} // namespace cli_param
