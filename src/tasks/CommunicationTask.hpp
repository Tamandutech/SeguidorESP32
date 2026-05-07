#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "cli/nimble-nordic-uart/nimble-nordic-uart.hpp"

class StateMachineTask;

struct CommEvent {
  enum class Kind : uint8_t {
    UartRxLine,
    OutgoingMessage,
    BleConnected,
    BleDisconnected,
  } kind;
  /// Valid length of `data` for text kinds; 0 for connection events.
  uint16_t len;
  char     data[2048];
};

/**
 * Active Object: one FreeRTOS task, one queue, BLE I/O and CLI handling in
 * that context only. NimBLE callbacks only enqueue; all nordic_uart_send
 * happens from run().
 */
class CommunicationTask {
public:
  /// Binds the task to the robot state machine used by CLI commands.
  explicit CommunicationTask(StateMachineTask *stateMachine);

  /// Creates and starts the FreeRTOS task pinned to the requested core.
  bool start(uint32_t stackSizeWords = 4096, UBaseType_t priority = 2,
             BaseType_t coreId = 0);

  /// Enqueue a formatted line for BLE notify (safe from any task; may drop on
  /// full queue).
  bool postOutgoingMessage(const char *fmt, ...)
      __attribute__((format(printf, 2, 3)));

  /// Generic event posting API; used by callbacks and external producers.
  bool post(const CommEvent &event, TickType_t timeoutTicks = 0);

  /// NimBLE / Nordic UART callbacks (non-blocking; only enqueues).
  void notifyBleRx(struct ble_gatt_access_ctxt *ctxt);
  void notifyBleStatus(enum nordic_uart_callback_type callbackType);

private:
  /// Static trampoline required by xTaskCreatePinnedToCore.
  static void taskEntry(void *param);
  /// Active Object main loop: initializes BLE and drains internal queue.
  void        run();

  /// Dispatches one queued communication event.
  void processEvent(const CommEvent &event);
  /// Parses and executes one CLI line received from BLE.
  void processIncomingLine(char *line);

  StateMachineTask *stateMachine_;
  QueueHandle_t     queue_;
  TaskHandle_t      taskHandle_;
};

/// Global hook for other subsystems to send messages/telemetry over BLE
/// (non-blocking).
bool commPushMessage(const char *fmt, ...) __attribute__((format(printf, 1, 2)));
