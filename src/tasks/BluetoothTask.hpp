#pragma once

#include <cstdint>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "cli/nimble-nordic-uart/nimble-nordic-uart.hpp"

class StateMachineTask;

struct BluetoothEvent {
  enum class Kind : uint8_t {
    UartRxLine,
    OutgoingMessage,
    BleConnected,
    BleDisconnected,
  } kind;
  /// Comprimento válido de `data` para tipos de texto; 0 para eventos de conexão.
  uint16_t len;
  char     data[2048];
};

/**
 * Active Object: uma tarefa FreeRTOS, uma fila, E/S BLE e tratamento da CLI
 * apenas nesse contexto. Callbacks NimBLE apenas inserem na fila; todo nordic_uart_send
 * ocorre a partir de run().
 */
class BluetoothTask {
public:
  /// Associa a tarefa à máquina de estados do robô usada pelos comandos da CLI.
  explicit BluetoothTask(StateMachineTask *stateMachine);

  /// Cria e inicia a tarefa FreeRTOS fixada no núcleo solicitado.
  bool start(uint32_t stackSizeWords = 4096, UBaseType_t priority = 2,
             BaseType_t coreId = 0);

  /// Insere na fila uma linha formatada para notify BLE (seguro de qualquer tarefa;
  /// pode descartar se a fila estiver cheia).
  bool postOutgoingMessage(const char *fmt, ...)
      __attribute__((format(printf, 2, 3)));

  /// API genérica para inserir eventos na fila; usada por callbacks e produtores externos.
  bool post(const BluetoothEvent &event, TickType_t timeoutTicks = 0);

  /// Callbacks NimBLE / Nordic UART (não bloqueiam; apenas inserem na fila).
  void notifyBleRx(struct ble_gatt_access_ctxt *ctxt);
  void notifyBleStatus(enum nordic_uart_callback_type callbackType);

private:
  /// Trampolim estático exigido por xTaskCreatePinnedToCore.
  static void taskEntry(void *param);
  /// Loop principal do Active Object: inicializa o BLE e esvazia a fila interna.
  void run();

  /// Despacha um evento bluetooth inserido na fila.
  void processEvent(const BluetoothEvent &event);
  /// Analisa e executa uma linha da CLI recebida por BLE.
  void processIncomingLine(char *line);

  StateMachineTask *stateMachine_;
  QueueHandle_t     queue_;
  TaskHandle_t      taskHandle_;
};

/// Gancho global para outros subsistemas enviarem mensagens/telemetria por BLE
/// (não bloqueante).
bool bluetoothPushMessage(const char *fmt, ...)
    __attribute__((format(printf, 1, 2)));
