#ifndef COMMUNICATION_TASK_HPP
#define COMMUNICATION_TASK_HPP

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include "nimble-nordic-uart/nimble-nordic-uart.h"

#include "cli/cli.hpp"
#include "context/GlobalData.hpp"
#include "context/RobotStateMachine.hpp"
#include "lib/CommunicationUtils.hpp"


// Callback para status de conexão da UART BLE
void uartStatusChangeCallback(enum nordic_uart_callback_type callback_type) {
  if(callback_type == NORDIC_UART_CONNECTED) {
    ESP_LOGI("CommunicationTask", "BLE UART connected");
  } else if(callback_type == NORDIC_UART_DISCONNECTED) {
    ESP_LOGI("CommunicationTask", "BLE UART disconnected");
  }
}

// Callback para receber dados da UART BLE
void uartReceiveCallback(struct ble_gatt_access_ctxt *ctxt) {
  // Get the actual data length from the mbuf
  uint16_t data_len = ctxt->om->om_len;

  // Create a null-terminated buffer to safely handle the data
  char   buffer[256];
  size_t copy_len =
      (data_len < sizeof(buffer) - 1) ? data_len : sizeof(buffer) - 1;
  memcpy(buffer, ctxt->om->om_data, copy_len);
  buffer[copy_len] = '\0';

  ESP_LOGI("CommunicationTask", "BLE UART received data: %s (len: %d)", buffer,
           data_len);

  // Execute CLI command and check return value
  int cliResult = cli(buffer);

  // If CLI returned an error, log and push error message to queue
  if(cliResult != CLI_SUCCESS) {
    switch(cliResult) {
    case CLI_ERROR_EMPTY_COMMAND:
      ESP_LOGE("CommunicationTask", "CLI Error: Empty command");
      pushMessageToQueue("Error: Empty command");
      break;
    case CLI_ERROR_COMMAND_NOT_FOUND:
      ESP_LOGE("CommunicationTask", "CLI Error: Command not found: %s", buffer);
      pushMessageToQueue("Error: Command not found: %s", buffer);
      break;
    case CLI_ERROR_TOO_MANY_ARGS:
      ESP_LOGE("CommunicationTask",
               "CLI Error: Too many arguments for command: %s", buffer);
      pushMessageToQueue("Error: Too many arguments");
      break;
    default:
      ESP_LOGE("CommunicationTask",
               "CLI Error: Unknown error (code: %d) for command: %s", cliResult,
               buffer);
      pushMessageToQueue("Error: Unknown error (code: %d)", cliResult);
      break;
    }
  }

  ESP_LOGI("CommunicationTask", "robotMode: %d",
           static_cast<int>(RobotStateMachine::get()));
}

// Processa mensagens da fila de comunicação
void processMessage(const Message &msg) {
  switch(msg.header.type) {
  case MessageType::LOG:
    ESP_LOGI("CommunicationTask", "MessageQueue: %s", msg.data.message);
    { nordic_uart_send(msg.data.message); }
    break;
  }
}

void communicationTaskLoop(void *params) {
  (void)params;
  nordic_uart_start("TT_SEMREH", uartStatusChangeCallback);
  nordic_uart_yield(uartReceiveCallback);

  Message receivedMessage;

  for(;;) {
    if(xQueueReceive(globalData.communicationQueue, &receivedMessage,
                     pdMS_TO_TICKS(100)) == pdTRUE) {
      processMessage(receivedMessage);
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#endif // COMMUNICATION_TASK_HPP
