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

  // Outgoing BLE/log queue. map_get drains to UART after each line from CLI;
  // extra depth helps MainTask bursts.
  globalData.communicationQueue = xQueueCreate(32, sizeof(Message));
  if(globalData.communicationQueue == NULL) {
    ESP_LOGE("Main", "Failed to create communication queue");
    return;
  }

  globalData.receivedUartMessages =
      xQueueCreate(10, sizeof(ReceivedUartMessage));
  if(globalData.receivedUartMessages == NULL) {
    ESP_LOGE("Main", "Failed to create receivedUartMessages queue");
    return;
  }

  // Mount FAT before any task runs so BLE param_set / map save see a mounted FS
  // (CommunicationTask can run before MainTask would otherwise mount here).
  {
    Storage  *storage      = Storage::getInstance();
    esp_err_t mount_result = storage->mount_storage("/data");
    if(mount_result != ESP_OK) {
      ESP_LOGE("Main",
               "FAT mount failed (%s); params.dat / map will not persist until "
               "fixed",
               esp_err_to_name(mount_result));
    }
  }

  // Stack sizes are in **words** (typically 4 bytes on ESP32-S3). Two stacks of
  // 60000 words (~240 KiB each) exceed the largest internal RAM heap block
  // (~283 KiB), so the second xTaskCreatePinnedToCore can fail silently and
  // MainTask never runs while BLE still works.
  constexpr UBaseType_t kCommTaskStackWords = 8192;  // 32 KiB
  constexpr UBaseType_t kMainTaskStackWords = 16384; // 64 KiB

  TaskHandle_t communicationTaskHandle = nullptr;
  if(xTaskCreatePinnedToCore(communicationTaskLoop, "CommunicationTask",
                             kCommTaskStackWords, NULL, 15,
                             &communicationTaskHandle, 0) != pdPASS) {
    ESP_LOGE("Main", "Failed to create CommunicationTask (heap/stack?)");
    return;
  }

  TaskHandle_t mainTaskHandle = nullptr;
  if(xTaskCreatePinnedToCore(mainTaskLoop, "MainTask", kMainTaskStackWords,
                             NULL, 16, &mainTaskHandle, 1) != pdPASS) {
    ESP_LOGE("Main", "Failed to create MainTask (heap/stack?)");
    return;
  }


  for(;;) {
    vTaskSuspend(NULL);
  }
}
