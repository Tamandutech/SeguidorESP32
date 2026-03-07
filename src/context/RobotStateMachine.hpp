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

    if(last == RobotState::MAPPING) {
      ESP_LOGI("RobotStateMachine", "Stopped at encoder milimeters: %d",
               globalData.mappingEncoderMilimetersAverage.load(
                   std::memory_order_relaxed));

      globalData.mapData.push_back(
          {.encoderMilimeters = globalData.mappingEncoderMilimetersAverage.load(
               std::memory_order_relaxed),
           .baseMotorPWM  = RobotEnv::MAPPING_MOTOR_PWM,
           .baseVacuumPWM = RobotEnv::BASE_VACUUM_PWM,
           .markType      = MapPoint::MarkType::STOP_COMMAND_MARK});
    }

    if(motorDriver == nullptr || vacuumDriver == nullptr) {
      ESP_LOGI("RobotStateMachine", "Invalid state transition to idle");
      setState(RobotState::IDLE);
      return true;
    }

    ESP_LOGI("RobotStateMachine", "Transitioning to idle");
    setState(RobotState::IDLE);
    motorDriver->pwmOutput(0, 0);
    vacuumDriver->pwmOutput(0);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
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
      ESP_LOGI("RobotStateMachine", "Transitioning to running");
      setState(RobotState::RUNNING);
      return true;
    }

    ESP_LOGI("RobotStateMachine", "Transitioning to running");
    setState(RobotState::RUNNING);
    encoderLeftDriver->clearCount();
    encoderRightDriver->clearCount();

    vTaskDelay(4000 / portTICK_PERIOD_MS);
    vacuumDriver->pwmOutput(globalData.mapData[0].baseVacuumPWM);
    vTaskDelay(1000);
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
      ESP_LOGI("RobotStateMachine", "Transitioning to mapping");
      setState(RobotState::MAPPING);
      globalData.mapData.clear();
      return true;
    }

    ESP_LOGI("RobotStateMachine", "Transitioning to mapping");
    setState(RobotState::MAPPING);
    globalData.mapData.clear();
    encoderLeftDriver->clearCount();
    encoderRightDriver->clearCount();

    return true;
  }
};

#endif // ROBOT_STATE_HPP
