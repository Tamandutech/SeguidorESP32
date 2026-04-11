#ifndef GLOBAL_DATA_CONTEXT_HPP
#define GLOBAL_DATA_CONTEXT_HPP

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <atomic>
#include <cstring>
#include <vector>

#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/IRSensorDriver/IRSensorDriver.hpp"
#include "drivers/LedRgbDriver/LedRgbDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"

// Message types for inter-task communication
enum class MessageType { LOG };

#define MESSAGE_LOG_NAME_SIZE    32
#define MESSAGE_LOG_MESSAGE_SIZE 2048

// Message structure for queue
struct Message {
  struct {
    MessageType type;
  } header;
  struct {
    char name[MESSAGE_LOG_NAME_SIZE];
    char message[MESSAGE_LOG_MESSAGE_SIZE];
  } data;
};

struct MapPoint {
  int32_t encoderMilimeters{};
  int32_t baseMotorPWM{};
  int32_t baseVacuumPWM{};
  enum MarkType {
    LEFT_MARK,
    RIGHT_MARK,
    HANDMADE_MARK,
    STOP_COMMAND_MARK,
    UNKNOWN_MARK
  } markType;
};

struct ParametersConfig {
  bool    runOnMappingMode{};
  int32_t vacuumPWM{};
};

struct GlobalData {
  // FreeRTOS queue for inter-task communication
  QueueHandle_t communicationQueue;


  /* Communication should only write on the variables below when the robot is in
   * IDLE mode */

  std::vector<MapPoint> mapData;

  std::atomic<int32_t> markCount = 0;

  std::atomic<int32_t> mappingEncoderMilimetersAverage = 0;

  /* Initialized in MainTask during calibration mode */

  ParametersConfig parametersConfig;

  MotorPins    motorPins   = {};
  MotorDriver *motorDriver = nullptr;

  IRSensorDriver *irSensorDriver = nullptr;

  EncoderDriver *encoderLeftDriver  = nullptr;
  EncoderDriver *encoderRightDriver = nullptr;

  VacuumPins    vacuumPins   = {};
  VacuumDriver *vacuumDriver = nullptr;

  LedRgbPins    ledRgbPins   = {};
  LedRgbDriver *ledRgbDriver = nullptr;

} static globalData;

#endif // GLOBAL_DATA_CONTEXT_HPP
