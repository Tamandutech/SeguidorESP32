// #ifndef LED_TASK_HPP
// #define LED_TASK_HPP

// #include "context/GlobalData.hpp"
// #include "context/RobotEnv.hpp"
// #include "drivers/LedRGBDriver/LedRgbDriver.hpp"
// #include "freertos/FreeRTOS.h"
// #include "freertos/queue.h"

// void ledTaskLoop(void *params) {
//   (void)params;
//   bool idleMode     = false;
//   bool idleLedWhite = true;
//   LedCommand cmd;

//   for(;;) {
//     if(globalData.ledCommandQueue == nullptr) {
//       vTaskDelay(pdMS_TO_TICKS(10));
//       continue;
//     }
//     if(idleMode && globalData.ledRgbDriver != nullptr) {
//       LedColor color = idleLedWhite ? LED_COLOR_WHITE : LED_COLOR_PURPLE;
//       for(uint8_t i = 0; i < RobotEnv::NUM_LED_STRIP_LEDS; i++) {
//         globalData.ledRgbDriver->setColor(i, color, 1.0f);
//       }
//       globalData.ledRgbDriver->refresh();
//       idleLedWhite = !idleLedWhite;
//     }

//     TickType_t timeout = idleMode ? pdMS_TO_TICKS(100) : portMAX_DELAY;
//     if(xQueueReceive(globalData.ledCommandQueue, &cmd, timeout) == pdTRUE) {
//       switch(cmd.type) {
//         case LedCommandType::ENTER_IDLE:
//           idleMode = true;
//           break;
//         case LedCommandType::EXIT_IDLE:
//           idleMode = false;
//           break;
//         case LedCommandType::BLINK_LED:
//           if(globalData.ledRgbDriver != nullptr) {
//             globalData.ledRgbDriver->setColor(cmd.ledIndex, cmd.color, 1.0f);
//             globalData.ledRgbDriver->refresh();
//             vTaskDelay(pdMS_TO_TICKS(150));
//             globalData.ledRgbDriver->setColor(cmd.ledIndex, LED_COLOR_BLACK,
//                                               1.0f);
//             globalData.ledRgbDriver->refresh();
//           }
//           break;
//         case LedCommandType::SET_ALL_LEDS:
//           if(globalData.ledRgbDriver != nullptr) {
//             for(uint8_t i = 0; i < RobotEnv::NUM_LED_STRIP_LEDS; i++) {
//               globalData.ledRgbDriver->setColor(i, cmd.color, 1.0f);
//             }
//             globalData.ledRgbDriver->refresh();
//           }
//           break;
//       }
//     }
//   }
// }

// #endif // LED_TASK_HPP
