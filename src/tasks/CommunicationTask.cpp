#include "CommunicationTask.hpp"

#include <cstdarg>
#include <cstring>

#include "esp_log.h"

#include "host/ble_gatt.h"

#include "tasks/cli/cli.hpp"
#include "tasks/StateMachineTask.hpp"

namespace {
const char *TAG = "CommunicationTask";

constexpr UBaseType_t kQueueDepth = 8;

CommunicationTask *g_commTask = nullptr;

// Bridges Nordic UART connection callbacks into the CommunicationTask queue.
void bleStatusCallback(enum nordic_uart_callback_type callbackType) {
  if(g_commTask != nullptr) {
    g_commTask->notifyBleStatus(callbackType);
  }
}

// Bridges Nordic UART RX callback into the CommunicationTask queue.
void bleRxCallback(struct ble_gatt_access_ctxt *ctxt) {
  if(g_commTask != nullptr) {
    g_commTask->notifyBleRx(ctxt);
  }
}
} // namespace

// Allocates the internal event queue; task thread is started by start().
CommunicationTask::CommunicationTask(StateMachineTask *stateMachine)
    : stateMachine_(stateMachine), queue_(nullptr), taskHandle_(nullptr) {
  queue_ = xQueueCreate(kQueueDepth, sizeof(CommEvent));
}

// Starts the Active Object task and registers this instance for C callbacks.
bool CommunicationTask::start(uint32_t stackSizeWords, UBaseType_t priority,
                              BaseType_t coreId) {
  if(queue_ == nullptr || taskHandle_ != nullptr || stateMachine_ == nullptr) {
    return false;
  }

  g_commTask = this;
  const BaseType_t ok =
      xTaskCreatePinnedToCore(&CommunicationTask::taskEntry, "comm_task",
                                stackSizeWords, this, priority, &taskHandle_,
                                coreId);
  if(ok != pdPASS) {
    g_commTask = nullptr;
    return false;
  }
  return true;
}

// Thread-safe event enqueue used by ISR-adjacent callbacks and app code.
bool CommunicationTask::post(const CommEvent &event, TickType_t timeoutTicks) {
  if(queue_ == nullptr) {
    return false;
  }
  return xQueueSend(queue_, &event, timeoutTicks) == pdTRUE;
}

// Formats one outbound message and enqueues it for BLE notify in AO context.
bool CommunicationTask::postOutgoingMessage(const char *fmt, ...) {
  CommEvent ev{};
  ev.kind = CommEvent::Kind::OutgoingMessage;
  va_list ap;
  va_start(ap, fmt);
  (void)vsnprintf(ev.data, sizeof(ev.data), fmt, ap);
  va_end(ap);
  ev.data[sizeof(ev.data) - 1] = '\0';
  ev.len = static_cast<uint16_t>(strlen(ev.data));
  return post(ev, 0);
}

// BLE RX callback handler: copies payload and enqueues a CLI line event.
void CommunicationTask::notifyBleRx(struct ble_gatt_access_ctxt *ctxt) {
  uint16_t    data_len = ctxt->om->om_len;
  CommEvent   ev{};
  ev.kind = CommEvent::Kind::UartRxLine;
  const size_t maxCopy = sizeof(ev.data) - 1U;
  const size_t dl      = static_cast<size_t>(data_len);
  const size_t copy_len = dl < maxCopy ? dl : maxCopy;
  memcpy(ev.data, ctxt->om->om_data, copy_len);
  ev.data[copy_len] = '\0';
  ev.len = static_cast<uint16_t>(strlen(ev.data));
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "comm queue full, dropping RX");
  }
}

// BLE connection state callback handler: enqueues connect/disconnect events.
void CommunicationTask::notifyBleStatus(enum nordic_uart_callback_type t) {
  CommEvent ev{};
  ev.kind = (t == NORDIC_UART_CONNECTED) ? CommEvent::Kind::BleConnected
                                          : CommEvent::Kind::BleDisconnected;
  ev.len  = 0;
  ev.data[0] = '\0';
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "comm queue full, dropping status");
  }
}

// FreeRTOS entry point trampoline.
void CommunicationTask::taskEntry(void *param) {
  auto *self = static_cast<CommunicationTask *>(param);
  self->run();
}

// AO runtime: initializes BLE UART and processes queued events forever.
void CommunicationTask::run() {
  if(nordic_uart_start("TT_SEMREH", bleStatusCallback) != ESP_OK) {
    ESP_LOGE(TAG, "nordic_uart_start failed");
    vTaskDelete(nullptr);
    return;
  }
  if(nordic_uart_yield(bleRxCallback) != ESP_OK) {
    ESP_LOGE(TAG, "nordic_uart_yield failed");
    vTaskDelete(nullptr);
    return;
  }

  CommEvent event{};
  while(true) {
    if(xQueueReceive(queue_, &event, portMAX_DELAY) == pdTRUE) {
      processEvent(event);
      while(xQueueReceive(queue_, &event, 0) == pdTRUE) {
        processEvent(event);
      }
    }
  }
}

// Handles one event: CLI input, outgoing notify, or connection logging.
void CommunicationTask::processEvent(const CommEvent &event) {
  switch(event.kind) {
  case CommEvent::Kind::UartRxLine: {
    char line[256];
    const size_t maxLine = sizeof(line) - 1U;
    const size_t el      = static_cast<size_t>(event.len);
    const size_t n       = el < maxLine ? el : maxLine;
    memcpy(line, event.data, n);
    line[n] = '\0';
    processIncomingLine(line);
    break;
  }
  case CommEvent::Kind::OutgoingMessage:
    (void)nordic_uart_send(event.data);
    break;
  case CommEvent::Kind::BleConnected:
    ESP_LOGI(TAG, "BLE UART connected");
    break;
  case CommEvent::Kind::BleDisconnected:
    ESP_LOGI(TAG, "BLE UART disconnected");
    break;
  }
}

// Runs CLI parser and emits protocol-compatible error responses.
void CommunicationTask::processIncomingLine(char *line) {
  const int cliResult = cli(line, stateMachine_);
  if(cliResult != CLI_SUCCESS) {
    switch(cliResult) {
    case CLI_ERROR_EMPTY_COMMAND:
      ESP_LOGE(TAG, "CLI Error: Empty command");
      (void)postOutgoingMessage("Error: Empty command\r\n");
      break;
    case CLI_ERROR_COMMAND_NOT_FOUND:
      ESP_LOGE(TAG, "CLI Error: Command not found / bad wire segment");
      (void)postOutgoingMessage("Error: Command not found\r\n");
      break;
    case CLI_ERROR_TOO_MANY_ARGS:
      ESP_LOGE(TAG, "CLI Error: Too many wire segments");
      (void)postOutgoingMessage("Error: Too many segments\r\n");
      break;
    default:
      ESP_LOGE(TAG, "CLI Error: Unknown error (code: %d)", cliResult);
      (void)postOutgoingMessage("Error: Unknown error (code: %d)\r\n", cliResult);
      break;
    }
  }
}

// Global helper used by command handlers to emit wire responses via AO queue.
bool commPushMessage(const char *fmt, ...) {
  if(g_commTask == nullptr) {
    return false;
  }
  CommEvent ev{};
  ev.kind = CommEvent::Kind::OutgoingMessage;
  va_list ap;
  va_start(ap, fmt);
  (void)vsnprintf(ev.data, sizeof(ev.data), fmt, ap);
  va_end(ap);
  ev.data[sizeof(ev.data) - 1] = '\0';
  ev.len = static_cast<uint16_t>(strlen(ev.data));
  return g_commTask->post(ev, 0);
}
