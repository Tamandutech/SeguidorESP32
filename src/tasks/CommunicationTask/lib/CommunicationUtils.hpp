#ifndef COMMUNICATION_UTILS_HPP
#define COMMUNICATION_UTILS_HPP

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"

#include "../nimble-nordic-uart/nimble-nordic-uart.h"
#include "context/GlobalData.hpp"

void pushMessageToQueue(const char *message, ...) {
  Message msg;
  msg.header.type = MessageType::LOG;

  // Initialize name field to empty string (not used anymore)
  msg.data.name[0] = '\0';

  // Format message using variadic arguments
  va_list args;
  va_start(args, message);
  vsnprintf(msg.data.message, MESSAGE_LOG_MESSAGE_SIZE, message, args);
  va_end(args);
  msg.data.message[MESSAGE_LOG_MESSAGE_SIZE - 1] = '\0';

  if(xQueueSend(globalData.communicationQueue, &msg, pdMS_TO_TICKS(100)) !=
     pdTRUE) {
    ESP_LOGW("MainTask", "Failed to send status update to communication queue");
  }
}

// Plain UTF-8 wire line(s) for BLE notify (no JSON wrapper).
void pushWireLineToQueue(const char *message, ...) {
  char buf[MESSAGE_LOG_MESSAGE_SIZE];
  va_list args;
  va_start(args, message);
  vsnprintf(buf, sizeof(buf), message, args);
  va_end(args);
  buf[sizeof(buf) - 1] = '\0';
  pushMessageToQueue("%s", buf);
}

#endif // COMMUNICATION_UTILS_HPP