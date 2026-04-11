// Framework
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Context
#include "context/GlobalData.hpp"
#include "context/RobotEnv.hpp"
// Non-volatile storage
#include "storage/storage.hpp"
// Tasks
#include "tasks/CommunicationTask/CommunicationTask.hpp"
#include "tasks/MainTask/MainTask.hpp"

extern "C" {
void app_main(void);
}

void app_main() {
  esp_log_level_set("QTRSensors", ESP_LOG_INFO);

  // Initialize the communication queue
  globalData.communicationQueue = xQueueCreate(10, sizeof(Message));
  if(globalData.communicationQueue == NULL) {
    ESP_LOGE("Main", "Failed to create communication queue");
    return;
  }

  // globalData.ledCommandQueue = xQueueCreate(10, sizeof(LedCommand));
  // if(globalData.ledCommandQueue == NULL) {
  //   ESP_LOGE("Main", "Failed to create LED command queue");
  //   return;
  // }

  // Storage::write(globalData.randomNumber.load(std::memory_order_relaxed));
  // Storage::write(globalData.randomChar.load(std::memory_order_relaxed));
  // Storage::write(globalData.randomFloat.load(std::memory_order_relaxed));
  // Storage::write(globalData.randomBool.load(std::memory_order_relaxed));


  // Task
  // 1 word = 4 bytes
  // The stack depth is setup to 60000 words, which is 240KB.
  TaskHandle_t communicationTaskHandle;
  xTaskCreatePinnedToCore(communicationTaskLoop, "CommunicationTask", 60000,
                          NULL, 15, &communicationTaskHandle,
                          0); // Run on Core 0
  TaskHandle_t mainTaskHandle;
  xTaskCreatePinnedToCore(mainTaskLoop, "MainTask", 60000, NULL, 16,
                          &mainTaskHandle, 1); // Run on Core 1


  for(;;) {
    vTaskSuspend(NULL);
  }
}
