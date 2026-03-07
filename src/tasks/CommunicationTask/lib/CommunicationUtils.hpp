#ifndef COMMUNICATION_UTILS_HPP
#define COMMUNICATION_UTILS_HPP

#include <cstdarg>
#include <cstdio>
#include <cstring>

#include "esp_log.h"

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

void pushDataJsonToQueue(const char *message, ...) {
  Message msg;
  msg.header.type = MessageType::LOG;

  msg.data.name[0] = '\0';

  va_list args;
  va_start(args, message);
  vsnprintf(msg.data.message, MESSAGE_LOG_MESSAGE_SIZE, message, args);
  va_end(args);
  msg.data.message[MESSAGE_LOG_MESSAGE_SIZE - 1] = '\0';

  // Substitute newline by literal "/n"
  char   buf[MESSAGE_LOG_MESSAGE_SIZE];
  size_t j = 0;
  for(size_t i = 0; msg.data.message[i] != '\0' && j < sizeof(buf) - 2; i++) {
    if(msg.data.message[i] == '\n') {
      buf[j++] = '\\';
      buf[j++] = 'n';
    } else {
      buf[j++] = msg.data.message[i];
    }
  }
  buf[j] = '\0';

  pushMessageToQueue("{\"data\": \"%s\"}", buf);
}

#endif // COMMUNICATION_UTILS_HPP