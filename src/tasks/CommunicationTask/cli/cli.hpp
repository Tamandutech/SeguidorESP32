#ifndef CLI_HPP
#define CLI_HPP

#include "esp_log.h"
#include <algorithm>
#include <cerrno>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <unordered_map>

#include "../lib/CommunicationUtils.hpp"
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

// Command handler function pointer type
typedef int (*CommandHandler)(int argc, char *argv[]);

// ========== Command Handler Functions ==========

// Parameter Commands
//
// param_list sends one outgoing JSON message per line (TamanduCLI / BLE), e.g.:
//   {"data": "Parameters: 2"}
//   {"data": "0 - State.runOnMappingMode: 0"}
//   {"data": "1 - Vacuum.speed: 0"}
//   {"data": "2 - Calibration.hardcodedCalibration: 0"}
//   {"data": "3 - PID.kP: 0.017000"}
//   {"data": "… - Mapping.mappingMotorPWM: 10"}
static int handleParamList(int argc, char *argv[]) {
  char valueState[64];
  char valueVacuum[64];
  char valueHardcodedCal[64];
  char valueMappingMotorPwm[64];
  char valuePidKp[64];
  char valuePidKi[64];
  char valuePidKd[64];
  bool hasState = getParameterValue("State", "runOnMappingMode", valueState,
                                    sizeof(valueState));
  bool hasVacuum =
      getParameterValue("Vacuum", "speed", valueVacuum, sizeof(valueVacuum));
  bool hasHardcodedCal =
      getParameterValue("Calibration", "hardcodedCalibration",
                        valueHardcodedCal, sizeof(valueHardcodedCal));
  bool hasMappingMotor =
      getParameterValue("Mapping", "mappingMotorPWM", valueMappingMotorPwm,
                        sizeof(valueMappingMotorPwm));
  bool hasPidKp =
      getParameterValue("PID", "kP", valuePidKp, sizeof(valuePidKp));
  bool hasPidKi =
      getParameterValue("PID", "kI", valuePidKi, sizeof(valuePidKi));
  bool hasPidKd =
      getParameterValue("PID", "kD", valuePidKd, sizeof(valuePidKd));
  int count = (hasState ? 1 : 0) + (hasVacuum ? 1 : 0) +
              (hasHardcodedCal ? 1 : 0) + (hasMappingMotor ? 1 : 0) +
              (hasPidKp ? 1 : 0) + (hasPidKi ? 1 : 0) + (hasPidKd ? 1 : 0);

  pushDataJsonToQueue("Parameters: %d", count);

  int index = 0;
  if(hasState) {
    pushDataJsonToQueue("%d - State.runOnMappingMode: %s", index++, valueState);
  }
  if(hasVacuum) {
    pushDataJsonToQueue("%d - Vacuum.speed: %s", index++, valueVacuum);
  }
  if(hasHardcodedCal) {
    pushDataJsonToQueue("%d - Calibration.hardcodedCalibration: %s", index++,
                        valueHardcodedCal);
  }
  if(hasMappingMotor) {
    pushDataJsonToQueue("%d - Mapping.mappingMotorPWM: %s", index++,
                        valueMappingMotorPwm);
  }
  if(hasPidKp) {
    pushDataJsonToQueue("%d - PID.kP: %s", index++, valuePidKp);
  }
  if(hasPidKi) {
    pushDataJsonToQueue("%d - PID.kI: %s", index++, valuePidKi);
  }
  if(hasPidKd) {
    pushDataJsonToQueue("%d - PID.kD: %s", index++, valuePidKd);
  }
  return CLI_SUCCESS;
}

static int handleParamGet(int argc, char *argv[]) {
  if(argc < 2) {
    ESP_LOGW("CLI", "param_get requires a parameter reference\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  ParsedReference ref;
  ParseError      parseError = parseClassNameParameter(argv[1], ref);
  if(parseError != ParseError::SUCCESS) {
    ESP_LOGW("CLI", "Invalid parameter format: %s\n", argv[1]);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  char value[64];
  if(getParameterValue(ref.className, ref.parameterName, value,
                       sizeof(value))) {
    pushDataJsonToQueue("%s", value);
    return CLI_SUCCESS;
  } else {
    ESP_LOGW("CLI", "Parameter not found: %s.%s\n", ref.className,
             ref.parameterName);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

static int handleParamSet(int argc, char *argv[]) {
  if(argc < 3) {
    ESP_LOGW("CLI", "param_set requires parameter reference and value\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  ParsedReference ref;
  ParseError      parseError = parseClassNameParameter(argv[1], ref);
  if(parseError != ParseError::SUCCESS) {
    ESP_LOGW("CLI", "Invalid parameter format: %s\n", argv[1]);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  if(setParameterValue(ref.className, ref.parameterName, argv[2])) {
    Storage *storage = Storage::getInstance();
    if(!storage->is_mounted()) {
      ESP_LOGW("CLI", "param_set: storage not mounted; RAM updated only");
      pushDataJsonToQueue("Error: storage not mounted");
      return CLI_SUCCESS;
    }
    if(storage->write(globalData.parametersConfig, PARAMETERS_CONFIG_FILE) !=
       ESP_OK) {
      ESP_LOGW("CLI", "Failed to save parameters to %s",
               PARAMETERS_CONFIG_FILE);
      pushDataJsonToQueue("Error: failed to save parameters");
      return CLI_SUCCESS;
    }
    pushDataJsonToQueue("OK");
    return CLI_SUCCESS;
  } else {
    ESP_LOGW("CLI", "Parameter not found or cannot be set: %s.%s\n",
             ref.className, ref.parameterName);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

// Mapping Commands


static int handleMapClear(int argc, char *argv[]) {
  globalData.mapData.clear();
  pushDataJsonToQueue("OK");
  return CLI_SUCCESS;
}

static int handleMapClearFlash(int argc, char *argv[]) {
  Storage              *storage = Storage::getInstance();
  std::vector<MapPoint> emptyMap;
  esp_err_t             ret = storage->write_vector(emptyMap, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE("CLI", "map_clearFlash: failed to clear Flash (%s)",
             esp_err_to_name(ret));
    pushDataJsonToQueue("Error: Failed to clear Flash");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  pushDataJsonToQueue("OK");
  return CLI_SUCCESS;
}

static int handleMapAdd(int argc, char *argv[]) {
  if(argc < 2) {
    ESP_LOGW("CLI", "map_add requires a payload\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }

  char payloadCopy[512];
  strncpy(payloadCopy, argv[1], sizeof(payloadCopy) - 1);
  payloadCopy[sizeof(payloadCopy) - 1] = '\0';

  char *record     = strtok(payloadCopy, ";");
  int   addedCount = 0;

  while(record != nullptr) {
    int id, vacuumPWM, encMedia, trackStatus, offset;
    if(sscanf(record, "%d,%d,%d,%d,%d", &id, &vacuumPWM, &encMedia,
              &trackStatus, &offset) == 5) {
      MapPoint point;
      point.encoderMilimeters = static_cast<int32_t>(encMedia);
      point.baseMotorPWM      = static_cast<int32_t>(offset);
      point.baseVacuumPWM     = static_cast<int32_t>(vacuumPWM);
      point.markType          = static_cast<MapPoint::MarkType>(trackStatus);
      globalData.mapData.push_back(point);
      addedCount++;
    }
    record = strtok(nullptr, ";");
  }

  if(addedCount > 0) {
    std::sort(globalData.mapData.begin(), globalData.mapData.end(),
              [](const MapPoint &a, const MapPoint &b) {
                return a.encoderMilimeters < b.encoderMilimeters;
              });
    pushDataJsonToQueue("OK");
    return CLI_SUCCESS;
  } else {
    ESP_LOGW("CLI", "Failed to parse any mapping records\n");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

static int handleMapSaveRuntime(int argc, char *argv[]) {
  Storage  *storage = Storage::getInstance();
  esp_err_t ret = storage->write_vector(globalData.mapData, MAP_STORAGE_FILE);
  if(ret != ESP_OK) {
    ESP_LOGE("CLI", "map_SaveRuntime: failed to save to Flash (%s)",
             esp_err_to_name(ret));
    pushDataJsonToQueue("Error: Failed to save to Flash");
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  pushDataJsonToQueue("OK");
  return CLI_SUCCESS;
}

static int handleMapGet(int argc, char *argv[]) {
  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    pushDataJsonToQueue("%zu,%d,%ld,%d,%ld", i, point.baseVacuumPWM,
                        static_cast<long>(point.encoderMilimeters),
                        static_cast<int>(point.markType),
                        static_cast<long>(point.baseMotorPWM));
    drainCommunicationQueueToBle();
  }
  return CLI_SUCCESS;
}

static int handleMapGetRuntime(int argc, char *argv[]) {
  std::string out;
  for(size_t i = 0; i < globalData.mapData.size(); i++) {
    const MapPoint &point = globalData.mapData[i];
    char            line[64];
    snprintf(line, sizeof(line), "%zu,%d,%ld,%d,%ld\n", i, 0,
             point.encoderMilimeters, point.markType, point.baseMotorPWM);
    out += line;
  }
  if(!out.empty()) {
    pushDataJsonToQueue("%s", out.c_str());
  }
  return CLI_SUCCESS;
}

// Runtime Commands
static int handleRuntimeList(int argc, char *argv[]) {
  std::string list;
  int         count = 0;
  char        value[64];
  if(getParameterValue("State", "runOnMappingMode", value, sizeof(value))) {
    char line[128];
    snprintf(line, sizeof(line), " %d - State.runOnMappingMode: %s\n", count++,
             value);
    list += line;
  }
  std::string out =
      "Runtime Parameters: " + std::to_string(count) + "\n" + list;
  pushDataJsonToQueue("%s", out.c_str());
  return CLI_SUCCESS;
}

// Control Commands
static int handlePause(int argc, char *argv[]) {
  RobotStateMachine::toIdle(globalData.motorDriver, globalData.vacuumDriver);
  return CLI_SUCCESS;
}

static int handleResume(int argc, char *argv[]) {
  if(globalData.parametersConfig.runOnMappingMode) {
    RobotStateMachine::toMapping(globalData.encoderLeftDriver,
                                 globalData.encoderRightDriver,
                                 globalData.vacuumDriver);
  } else {
    RobotStateMachine::toRunning(globalData.encoderLeftDriver,
                                 globalData.encoderRightDriver,
                                 globalData.vacuumDriver);
  }
  return CLI_SUCCESS;
}

// Initialize ADC for battery voltage reading (static handle)
static adc_oneshot_unit_handle_t getBatteryAdcHandle() {
  static adc_oneshot_unit_handle_t adc2_handle = nullptr;
  static bool                      initialized = false;

  if(!initialized) {
    // GPIO 18 is on ADC2 for ESP32-S3
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

    // Configure ADC channel for GPIO 18 (ADC2_CHANNEL_7 on ESP32-S3)
    adc_oneshot_chan_cfg_t config = {
        .atten    = ADC_ATTEN_DB_12, // 0-3.3V range
        .bitwidth = ADC_BITWIDTH_12, // 12-bit resolution (0-4095)
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

static int handleBatVoltage(int argc, char *argv[]) {
  adc_oneshot_unit_handle_t adc_handle = getBatteryAdcHandle();
  if(adc_handle == nullptr) {
    pushDataJsonToQueue("0.0");
    return CLI_SUCCESS;
  }

  int       adc_raw = 0;
  esp_err_t ret     = adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_raw);

  if(ret != ESP_OK) {
    ESP_LOGE("CLI", "Failed to read battery voltage ADC: %s",
             esp_err_to_name(ret));
    pushDataJsonToQueue("0.0");
    return CLI_SUCCESS;
  }

  uint32_t voltage_mv = (static_cast<uint32_t>(adc_raw) * 3300) / 4095;

  pushDataJsonToQueue("%d.0", voltage_mv);
  return CLI_SUCCESS;
}

// Command map initialization function
static std::unordered_map<std::string, CommandHandler> &getCommandMap() {
  static std::unordered_map<std::string, CommandHandler> commandMap = {
      // Parameter Commands
      {"param_list",      handleParamList     },
      {"param_get",       handleParamGet      },
      {"param_set",       handleParamSet      },
      // Mapping Commands
      {"map_clear",       handleMapClear      },
      {"map_clearFlash",  handleMapClearFlash },
      {"map_add",         handleMapAdd        },
      {"map_SaveRuntime", handleMapSaveRuntime},
      {"map_get",         handleMapGet        },
      {"map_getRuntime",  handleMapGetRuntime },
      // Runtime Commands
      {"runtime_list",    handleRuntimeList   },
      // Control Commands
      {"pause",           handlePause         },
      {"resume",          handleResume        },
      {"bat_voltage",     handleBatVoltage    },
  };
  return commandMap;
}

int cli(char *command) {
  const int MAX_ARGS = 16;
  char     *argv[MAX_ARGS];
  int       argc = 0;

  char *token = strtok(command, " \t\n\r");
  while(token != nullptr && argc < MAX_ARGS) {
    argv[argc] = token;
    argc++;
    token = strtok(nullptr, " \t\n\r");
  }

  if(token != nullptr) {
    ESP_LOGW("CLI", "Too many arguments, max %d\n", MAX_ARGS);
    return CLI_ERROR_TOO_MANY_ARGS;
  }

  if(argc < MAX_ARGS) {
    argv[argc] = nullptr;
  }

  if(argc == 0) {
    ESP_LOGI("CLI", "Empty command\n");
    return CLI_ERROR_EMPTY_COMMAND;
  }

  std::unordered_map<std::string, CommandHandler> &commandMap = getCommandMap();
  auto it = commandMap.find(std::string(argv[0]));

  if(it != commandMap.end()) {
    ESP_LOGI("CLI", "Command found: %s\n", argv[0]);
    return it->second(argc, argv);
  } else {
    ESP_LOGI("CLI", "Command not found: %s\n", argv[0]);
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
}

#endif // CLI_HPP