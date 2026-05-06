#pragma once

#include <stdint.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

enum class EventType { START, STOP, CALIBRATE, MAP, CALIBRATION_DONE, ERROR };

struct Event {
  EventType type;
};

enum class RobotState { IDLE, CALIBRATING, RUNNING, MAPPING };

extern volatile RobotState gRobotState;

class StateMachineTask {
public:
  StateMachineTask();
  ~StateMachineTask();

  bool start(uint32_t stackSizeWords = 2048, UBaseType_t priority = 3,
             BaseType_t coreId = 0);
  bool postEvent(const Event &event, TickType_t timeoutTicks = 0);
  RobotState getState() const;

private:
  static void taskEntry(void *param);
  void        run();

  void handleEvent(const Event &event);
  void transitionTo(RobotState newState);

  // State handlers
  void onIdle(const Event &event);
  void onCalibrating(const Event &event);
  void onRunning(const Event &event);
  void onMapping(const Event &event);

private:
  QueueHandle_t queue;
  TaskHandle_t  taskHandle;
  RobotState    currentState;
};