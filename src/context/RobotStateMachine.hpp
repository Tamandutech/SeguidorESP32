#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include <atomic>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "context/GlobalData.hpp"
#include "context/RobotEnv.hpp"
#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"

enum class RobotState { CALIBRATION, IDLE, RUNNING, MAPPING };

class RobotStateMachine {
private:
  static void setState(RobotState s) {
    state_.store(s, std::memory_order_relaxed);
  }

  static inline std::atomic<RobotState> state_{RobotState::IDLE};

public:
  static RobotState get() { return state_.load(std::memory_order_relaxed); }

  static bool toCalibration() {
    setState(RobotState::CALIBRATION);
    return true;
  }

  static bool toIdle(MotorDriver *motorDriver, VacuumDriver *vacuumDriver) {
    RobotState last = get();
    if(last == RobotState::IDLE) {
      ESP_LOGI("RobotStateMachine", "Invalid state transition to idle");
      return false;
    }

    ESP_LOGI("RobotStateMachine", "Transitioning to idle");
    setState(RobotState::IDLE);
    motorDriver->pwmOutput(0, 0);
    vacuumDriver->pwmOutput(0);
    // if(globalData.ledCommandQueue != nullptr) {
    //   LedCommand cmd = {.type     = LedCommandType::ENTER_IDLE,
    //                     .ledIndex = 0,
    //                     .color    = LED_COLOR_BLACK};
    //   xQueueSend(globalData.ledCommandQueue, &cmd, 0);
    // }
    return true;
  }

  static bool toRunning(EncoderDriver *encoderLeftDriver,
                        EncoderDriver *encoderRightDriver,
                        VacuumDriver  *vacuumDriver) {
    RobotState last = get();
    if(last == RobotState::RUNNING || last == RobotState::CALIBRATION) {
      ESP_LOGI("RobotStateMachine", "Invalid state transition to running");
      return false;
    }
    if(encoderLeftDriver == nullptr || encoderRightDriver == nullptr ||
       vacuumDriver == nullptr) {
      ESP_LOGI("RobotStateMachine", "Driver not initialized");
      return false;
    }

    ESP_LOGI("RobotStateMachine", "Transitioning to running");
    encoderLeftDriver->clearCount();
    encoderRightDriver->clearCount();

    vTaskDelay(3000 / portTICK_PERIOD_MS);
    // if(last == RobotState::IDLE && globalData.ledCommandQueue != nullptr) {
    //   LedCommand cmd = {.type     = LedCommandType::EXIT_IDLE,
    //                     .ledIndex = 0,
    //                     .color    = LED_COLOR_BLACK};
    //   xQueueSend(globalData.ledCommandQueue, &cmd, 0);
    // }
    setState(RobotState::RUNNING);
    // if(globalData.ledCommandQueue != nullptr) {
    //   LedCommand cmd = {.type     = LedCommandType::SET_ALL_LEDS,
    //                     .ledIndex = 0,
    //                     .color    = LED_COLOR_GREEN};
    //   xQueueSend(globalData.ledCommandQueue, &cmd, 0);
    // }
    return true;
  }

  static bool toMapping(EncoderDriver *encoderLeftDriver,
                        EncoderDriver *encoderRightDriver,
                        VacuumDriver  *vacuumDriver) {
    RobotState last = get();
    if(last == RobotState::MAPPING || last == RobotState::RUNNING ||
       last == RobotState::CALIBRATION) {
      ESP_LOGI("RobotStateMachine", "Invalid state transition to mapping");
      return false;
    }

    if(encoderLeftDriver == nullptr || encoderRightDriver == nullptr ||
       vacuumDriver == nullptr) {
      ESP_LOGI("RobotStateMachine", "Driver not initialized");
      return false;
    }

    ESP_LOGI("RobotStateMachine", "Transitioning to mapping");
    globalData.mapData.clear();
    encoderLeftDriver->clearCount();
    encoderRightDriver->clearCount();
    // if(last == RobotState::IDLE && globalData.ledCommandQueue != nullptr) {
    //   LedCommand cmd = {.type     = LedCommandType::EXIT_IDLE,
    //                     .ledIndex = 0,
    //                     .color    = LED_COLOR_BLACK};
    //   xQueueSend(globalData.ledCommandQueue, &cmd, 0);
    // }
    setState(RobotState::MAPPING);

    return true;
  }
};

#endif // ROBOT_STATE_HPP
