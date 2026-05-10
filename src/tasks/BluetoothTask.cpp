#include "BluetoothTask.hpp"

#include <cstdarg>
#include <cstring>

#include "esp_log.h"

#include "host/ble_gatt.h"

#include "tasks/StateMachineTask.hpp"
#include "tasks/cli/cli.hpp"

namespace {
const char *TAG = "BluetoothTask";

constexpr UBaseType_t kQueueDepth = 8;

BluetoothTask *g_bluetoothTask = nullptr;

// Encaminha os callbacks de conexão do Nordic UART para a fila do BluetoothTask.
void bleStatusCallback(enum nordic_uart_callback_type callbackType) {
  if(g_bluetoothTask != nullptr) {
    g_bluetoothTask->notifyBleStatus(callbackType);
  }
}

// Encaminha o callback de RX do Nordic UART para a fila do BluetoothTask.
void bleRxCallback(struct ble_gatt_access_ctxt *ctxt) {
  if(g_bluetoothTask != nullptr) {
    g_bluetoothTask->notifyBleRx(ctxt);
  }
}
} // namespace

// Aloca a fila interna de eventos; a thread da tarefa é iniciada por start().
BluetoothTask::BluetoothTask(StateMachineTask *stateMachine)
    : stateMachine_(stateMachine), queue_(nullptr), taskHandle_(nullptr) {
  queue_ = xQueueCreate(kQueueDepth, sizeof(BluetoothEvent));
}

// Inicia a tarefa Active Object e registra esta instância para callbacks em C.
bool BluetoothTask::start(uint32_t stackSizeWords, UBaseType_t priority,
                          BaseType_t coreId) {
  if(queue_ == nullptr || taskHandle_ != nullptr || stateMachine_ == nullptr) {
    return false;
  }

  g_bluetoothTask     = this;
  const BaseType_t ok = xTaskCreatePinnedToCore(
      &BluetoothTask::taskEntry, "bluetooth_task", stackSizeWords, this,
      priority, &taskHandle_, coreId);
  if(ok != pdPASS) {
    g_bluetoothTask = nullptr;
    return false;
  }
  return true;
}

// Inserção na fila thread-safe usada por callbacks próximos a ISR e pelo código da aplicação.
bool BluetoothTask::post(const BluetoothEvent &event, TickType_t timeoutTicks) {
  if(queue_ == nullptr) {
    return false;
  }
  return xQueueSend(queue_, &event, timeoutTicks) == pdTRUE;
}

// Formata uma mensagem de saída e insere na fila para notify BLE no contexto do
// BluetoothTask.
bool BluetoothTask::postOutgoingMessage(const char *fmt, ...) {
  BluetoothEvent ev{};
  ev.kind = BluetoothEvent::Kind::OutgoingMessage;
  va_list ap;
  va_start(ap, fmt);
  (void)vsnprintf(ev.data, sizeof(ev.data), fmt, ap);
  va_end(ap);
  ev.data[sizeof(ev.data) - 1] = '\0';
  ev.len                       = static_cast<uint16_t>(strlen(ev.data));
  return post(ev, 0);
}

// Manipulador do callback de RX BLE: copia o payload e insere na fila um evento de linha da CLI.
void BluetoothTask::notifyBleRx(struct ble_gatt_access_ctxt *ctxt) {
  uint16_t       data_len = ctxt->om->om_len;
  BluetoothEvent ev{};
  ev.kind               = BluetoothEvent::Kind::UartRxLine;
  const size_t maxCopy  = sizeof(ev.data) - 1U;
  const size_t dl       = static_cast<size_t>(data_len);
  const size_t copy_len = dl < maxCopy ? dl : maxCopy;
  memcpy(ev.data, ctxt->om->om_data, copy_len);
  ev.data[copy_len] = '\0';
  ev.len            = static_cast<uint16_t>(strlen(ev.data));
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "bluetooth queue full, dropping RX");
  }
}

// Manipulador do callback de estado da conexão BLE: insere na fila eventos de conexão/desconexão.
void BluetoothTask::notifyBleStatus(enum nordic_uart_callback_type t) {
  BluetoothEvent ev{};
  ev.kind    = (t == NORDIC_UART_CONNECTED)
                 ? BluetoothEvent::Kind::BleConnected
                 : BluetoothEvent::Kind::BleDisconnected;
  ev.len     = 0;
  ev.data[0] = '\0';
  if(!post(ev, 0)) {
    ESP_LOGW(TAG, "bluetooth queue full, dropping status");
  }
}

// Trampolim de entrada do FreeRTOS.
void BluetoothTask::taskEntry(void *param) {
  auto *self = static_cast<BluetoothTask *>(param);
  self->run();
}

// Tempo de execução do Active Object: inicializa o UART BLE e processa indefinidamente os eventos inseridos na fila.
void BluetoothTask::run() {
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

  BluetoothEvent event{};
  while(true) {
    if(xQueueReceive(queue_, &event, portMAX_DELAY) == pdTRUE) {
      processEvent(event);
      while(xQueueReceive(queue_, &event, 0) == pdTRUE) {
        processEvent(event);
      }
    }
  }
}

// Trata um evento: entrada da CLI, notify de saída ou registro de conexão.
void BluetoothTask::processEvent(const BluetoothEvent &event) {
  switch(event.kind) {
  case BluetoothEvent::Kind::UartRxLine: {
    char         line[256];
    const size_t maxLine = sizeof(line) - 1U;
    const size_t el      = static_cast<size_t>(event.len);
    const size_t n       = el < maxLine ? el : maxLine;
    memcpy(line, event.data, n);
    line[n] = '\0';
    processIncomingLine(line);
    break;
  }
  case BluetoothEvent::Kind::OutgoingMessage:
    (void)nordic_uart_send(event.data);
    break;
  case BluetoothEvent::Kind::BleConnected:
    ESP_LOGI(TAG, "BLE UART connected");
    break;
  case BluetoothEvent::Kind::BleDisconnected:
    ESP_LOGI(TAG, "BLE UART disconnected");
    break;
  }
}

// Executa o parser da CLI e emite respostas de erro compatíveis com o protocolo.
void BluetoothTask::processIncomingLine(char *line) {
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
      (void)postOutgoingMessage("Error: Unknown error (code: %d)\r\n",
                                cliResult);
      break;
    }
  }
}

// Auxiliar global usado pelos manipuladores de comando para emitir respostas na linha via fila do Active Object.
bool bluetoothPushMessage(const char *fmt, ...) {
  if(g_bluetoothTask == nullptr) {
    return false;
  }
  BluetoothEvent ev{};
  ev.kind = BluetoothEvent::Kind::OutgoingMessage;
  va_list ap;
  va_start(ap, fmt);
  (void)vsnprintf(ev.data, sizeof(ev.data), fmt, ap);
  va_end(ap);
  ev.data[sizeof(ev.data) - 1] = '\0';
  ev.len                       = static_cast<uint16_t>(strlen(ev.data));
  return g_bluetoothTask->post(ev, 0);
}
