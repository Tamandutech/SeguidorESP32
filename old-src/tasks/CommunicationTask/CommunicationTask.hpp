#ifndef COMMUNICATION_TASK_HPP
#define COMMUNICATION_TASK_HPP

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <string>

#include "esp_log.h"

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

// NimBLE GATT callback: only enqueue — CLI runs in communicationTaskLoop.
void uartReceiveCallback(struct ble_gatt_access_ctxt *ctxt) {
  uint16_t data_len = ctxt->om->om_len;

  ReceivedUartMessage uartMsg{};
  size_t              copy_len = (data_len < RECEIVED_UART_MESSAGE_SIZE - 1)
                                   ? data_len
                                   : RECEIVED_UART_MESSAGE_SIZE - 1;
  memcpy(uartMsg.text, ctxt->om->om_data, copy_len);
  uartMsg.text[copy_len] = '\0';

  if(xQueueSend(globalData.receivedUartMessages, &uartMsg, 0) != pdTRUE) {
    ESP_LOGI("CommunicationTask",
             "receivedUartMessages full, dropping (len=%u)",
             static_cast<unsigned>(data_len));
  }
}

static void processReceivedUartLine(char *buffer) {
  int cliResult = cli(buffer);

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

static void drainOutgoingMessages() {
  Message receivedMessage;
  while(xQueueReceive(globalData.communicationQueue, &receivedMessage, 0) ==
        pdTRUE) {
    processMessage(receivedMessage);
  }
}

void communicationTaskLoop(void *params) {
  (void)params;
  nordic_uart_start("TT_SEMREH", uartStatusChangeCallback);
  nordic_uart_yield(uartReceiveCallback);

  for(;;) {
    // Keep large stack objects in narrow scopes so BLE RX + CLI + map_get +
    // drainOutgoingMessages + NimBLE notify do not overlap two ~2 KiB Messages.
    {
      ReceivedUartMessage uartMsg;
      if(xQueueReceive(globalData.receivedUartMessages, &uartMsg,
                       pdMS_TO_TICKS(100)) == pdTRUE) {

        ESP_LOGI("CommunicationTask", "BLE UART received data: %s",
                 uartMsg.text);
        processReceivedUartLine(uartMsg.text);
        drainOutgoingMessages();
      }
    }

    {
      Message receivedMessage;
      if(xQueueReceive(globalData.communicationQueue, &receivedMessage,
                       pdMS_TO_TICKS(100)) == pdTRUE) {
        processMessage(receivedMessage);
        while(xQueueReceive(globalData.communicationQueue, &receivedMessage,
                            0) == pdTRUE) {
          processMessage(receivedMessage);
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

#endif // COMMUNICATION_TASK_HPP
