// Framework
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Helpers
#include "context/GlobalData.hpp"
#include "data_types.hpp"
#include "storage/storage.hpp"
#include "tasks/cli/cli.hpp"
// Tasks
#include "tasks/BluetoothTask.hpp"
#include "tasks/ControlTask.hpp"
#include "tasks/StateMachineTask.hpp"

extern "C" {
void app_main(void);
}

namespace {
StateMachineTask gStateMachineTask;
BluetoothTask    gBluetoothTask(&gStateMachineTask);
} // namespace

void app_main() {
  Storage *storage = Storage::getInstance();
  if(storage->mount_storage("/data") == ESP_OK) {
    (void)storage->read(globalData.parametersConfig, PARAMETERS_STORAGE_FILE);
    (void)storage->read_vector(globalData.mapData, MAP_STORAGE_FILE);
  }

  // TASK CREATION START
  // Stack sizes are in **words** (typically 4 bytes on ESP32-S3).
  // Largest internal RAM heap block is(~283 KiB)
  // State machine (Core 0, medium priority).
  (void)gStateMachineTask.start(2048, 3, 0);
  // BLE + CLI active object (Core 0, lower priority than FSM; NimBLE host is
  // pinned to core 0 in sdkconfig).
  (void)gBluetoothTask.start(4096, 2, 0);

  // TASK CREATION END

  for(;;) {
    vTaskSuspend(NULL);
  }
}
