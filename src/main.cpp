// Framework
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// Context
#include "data_types.hpp"
// Tasks
#include "tasks/CommunicationTask.hpp"
#include "tasks/ControlTask.hpp"
#include "tasks/StateMachineTask.hpp"

// Global Constants
#define ROBOT_WIDTH                 (4)
#define WHEEL_RADIUS                (11)
#define WHEEL_CIRCUMFERENCE         (70)
#define ENCODER_PULSES_PER_ROTATION (4095)

#define MAPPING_MOTOR_PWM (10)
#define BASE_VACUUM_PWM   (100)

#define MAX_MOTOR_PWM (66)

#define SIDE_SENSOR_READ_AVERAGE_COUNT (5)

#define EPSILON_TOLERANCE \
  (1e-6F) // Tolerância para comparações de ponto flutuante

// Constantes para prevenção de integral windup no PID
#define INTEGRAL_MAX (1000.0F)  // Valor máximo para o termo integral
#define INTEGRAL_MIN (-1000.0F) // Valor mínimo para o termo integral

#define GPIO_LED_DEBUG (47)
#define NUM_LEDS_DEBUG (4)

#define GPIO_BATTERY_VOLTAGE (18)

#define GPIO_DIRECTION_A (9)
#define GPIO_DIRECTION_B (37)
#define GPIO_PWM_A       (3)
#define GPIO_PWM_B       (38)

#define GPIO_PWM_VACUUM (11)

#define GPIO_ENCODER_LEFT_A  (7)
#define GPIO_ENCODER_LEFT_B  (6)
#define GPIO_ENCODER_RIGHT_A (12)
#define GPIO_ENCODER_RIGHT_B (13)

#define GPIO_MULTIPLEXER_DIGITAL_ADDRESS {39, 40, 41, 42}
#define GPIO_MULTIPLEXER_ANALOG_INPUT    (10)
#define GPIO_MULTIPLEXER_LINE_SENSORS_INDEX \
  {13, 12, 11, 10, 9, 8, 5, 4, 3, 2, 1, 0}
#define GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX {15, 14, 6, 7}


extern "C" {
void app_main(void);
}

namespace {
// Static lifetime active object for robot FSM.
StateMachineTask gStateMachineTask;
} // namespace

void app_main() {
  // SETUP START
  // Queue and task handle are initialized in constructor.

  // SETUP END

  // TASK CREATION START
  // Stack sizes are in **words** (typically 4 bytes on ESP32-S3).
  // Largest internal RAM heap block is(~283 KiB)
  // State machine (Core 0, low-medium priority).
  (void)gStateMachineTask.start(2048, 3, 0);

  // TASK CREATION END

  for(;;) {
    vTaskSuspend(NULL);
  }
}
