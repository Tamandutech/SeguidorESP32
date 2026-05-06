#include "StateMachineTask.hpp"

#define QUEUE_SIZE 10

volatile RobotState gRobotState = RobotState::IDLE;

StateMachineTask::StateMachineTask()
    : queue(nullptr), taskHandle(nullptr), currentState(RobotState::IDLE) {
  queue = xQueueCreate(QUEUE_SIZE, sizeof(Event));
}

StateMachineTask::~StateMachineTask() {
  if(taskHandle != nullptr) {
    vTaskDelete(taskHandle);
    taskHandle = nullptr;
  }

  if(queue != nullptr) {
    vQueueDelete(queue);
    queue = nullptr;
  }
}

bool StateMachineTask::start(uint32_t stackSizeWords, UBaseType_t priority,
                             BaseType_t coreId) {
  if(queue == nullptr || taskHandle != nullptr) {
    return false;
  }

  return xTaskCreatePinnedToCore(&StateMachineTask::taskEntry, "robot_task",
                                 stackSizeWords, this, priority, &taskHandle,
                                 coreId) == pdPASS;
}

bool StateMachineTask::postEvent(const Event &event, TickType_t timeoutTicks) {
  if(queue == nullptr) {
    return false;
  }

  return xQueueSend(queue, &event, timeoutTicks) == pdTRUE;
}

RobotState StateMachineTask::getState() const { return currentState; }

void StateMachineTask::taskEntry(void *param) {
  auto *self = static_cast<StateMachineTask *>(param);
  self->run();
}

void StateMachineTask::run() {
  Event event;
  gRobotState = currentState;

  while(true) {
    if(xQueueReceive(queue, &event, portMAX_DELAY)) {
      handleEvent(event);
    }
  }
}

void StateMachineTask::handleEvent(const Event &event) {
  if(event.type == EventType::ERROR) {
    transitionTo(RobotState::IDLE);
    return;
  }

  switch(currentState) {
  case RobotState::IDLE: onIdle(event); break;
  case RobotState::CALIBRATING: onCalibrating(event); break;
  case RobotState::RUNNING: onRunning(event); break;
  case RobotState::MAPPING: onMapping(event); break;
  }
}

void StateMachineTask::transitionTo(RobotState newState) {
  if(currentState == newState) {
    return;
  }

  currentState = newState;
  gRobotState = newState;
}

//
// STATE: IDLE
//
void StateMachineTask::onIdle(const Event &event) {
  switch(event.type) {
  case EventType::CALIBRATE: transitionTo(RobotState::CALIBRATING); break;
  case EventType::START: transitionTo(RobotState::RUNNING); break;
  case EventType::MAP: transitionTo(RobotState::MAPPING); break;
  case EventType::STOP:
  case EventType::CALIBRATION_DONE:
  case EventType::ERROR: break;
  default: break;
  }
}

//
// STATE: CALIBRATING
//
void StateMachineTask::onCalibrating(const Event &event) {
  switch(event.type) {
  case EventType::CALIBRATION_DONE: transitionTo(RobotState::IDLE); break;
  case EventType::STOP: transitionTo(RobotState::IDLE); break;
  case EventType::START:
  case EventType::MAP:
  case EventType::CALIBRATE:
  case EventType::ERROR: break;
  default: break;
  }
}

//
// STATE: RUNNING
//
void StateMachineTask::onRunning(const Event &event) {
  switch(event.type) {
  case EventType::STOP: transitionTo(RobotState::IDLE); break;
  case EventType::MAP: transitionTo(RobotState::MAPPING); break;
  case EventType::CALIBRATE:
  case EventType::CALIBRATION_DONE:
  case EventType::START:
  case EventType::ERROR: break;
  default: break;
  }
}

//
// STATE: MAPPING
//
void StateMachineTask::onMapping(const Event &event) {
  switch(event.type) {
  case EventType::STOP: transitionTo(RobotState::IDLE); break;
  case EventType::START: transitionTo(RobotState::RUNNING); break;
  case EventType::CALIBRATE:
  case EventType::CALIBRATION_DONE:
  case EventType::MAP:
  case EventType::ERROR: break;
  default: break;
  }
}