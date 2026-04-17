#ifndef MAIN_TASK_HPP
#define MAIN_TASK_HPP

#include "context/GlobalData.hpp"
#include "context/RobotStateMachine.hpp"

#include "drivers/EncoderDriver/EncoderDriver.hpp"
#include "drivers/IRSensorDriver/IRSensorDriver.hpp"
#include "drivers/LedRgbDriver/LedRgbDriver.hpp"
#include "drivers/MotorDriver/MotorDriver.hpp"
#include "drivers/VacuumDriver/VacuumDriver.hpp"

#include "storage/storage.hpp"

#include "tasks/MainTask/PathController/PathController.hpp"


void mainTaskLoop(void *params) {
  (void)params;
  RobotStateMachine::toCalibration();

  // Initialize storage and load configuration
  Storage  *storage      = Storage::getInstance();
  esp_err_t mount_result = storage->mount_storage("/data");
  if(mount_result != ESP_OK) {
    ESP_LOGW("MainTask", "Failed to mount storage, using hardcoded defaults");
  }

  // Load parametersConfig from storage, use defaults if file doesn't exist
  ParametersConfig default_parameters_config = {.runOnMappingMode = false,
                                                .vacuumPWM        = 0};
  if(storage->file_exists("params.dat")) {
    esp_err_t read_result =
        storage->read(globalData.parametersConfig, "params.dat");
    if(read_result != ESP_OK) {
      ESP_LOGW("MainTask", "Failed to read parametersConfig, using defaults");
      globalData.parametersConfig = default_parameters_config;
    } else {
      ESP_LOGI("MainTask", "Loaded parametersConfig from storage");
    }
  } else {
    ESP_LOGI("MainTask", "parametersConfig file not found, using defaults");
    globalData.parametersConfig = default_parameters_config;
  }

  // Load mapData from storage, use defaults if file doesn't exist
  std::vector<MapPoint> default_map_data = {
      {.encoderMilimeters = 0,
       .baseMotorPWM      = 15,
       .baseVacuumPWM     = 100,
       .markType          = MapPoint::MarkType::UNKNOWN_MARK},
      {.encoderMilimeters = 66000,
       .baseMotorPWM      = 15,
       .baseVacuumPWM     = 100,
       .markType          = MapPoint::MarkType::UNKNOWN_MARK},
  };
  if(storage->file_exists("map_data.dat")) {
    esp_err_t read_result =
        storage->read_vector(globalData.mapData, "map_data.dat");
    if(read_result != ESP_OK) {
      ESP_LOGW("MainTask", "Failed to read mapData, using defaults");
      globalData.mapData = default_map_data;
    } else {
      ESP_LOGI("MainTask", "Loaded mapData from storage (%zu points)",
               globalData.mapData.size());
    }
  } else {
    ESP_LOGI("MainTask", "mapData file not found, using defaults");
    globalData.mapData = default_map_data;
  }

  // Set finishLineCount to the last point's encoderMilimeters from mapData
  int32_t finishLineCount = 0;
  if(!globalData.mapData.empty()) {
    finishLineCount = globalData.mapData.back().encoderMilimeters;
    ESP_LOGI("MainTask", "Set finishLineCount to %ld (last mapData point)",
             finishLineCount);
  } else {
    ESP_LOGW("MainTask", "mapData is empty, finishLineCount set to 0");
  }

  // uint16_t rawSensorValues[16];
  uint16_t sideSensorValues[4];
  uint16_t lineSensorValues[12];

  // Initialize pins and drivers in globalData during calibration mode
  if(globalData.motorDriver == nullptr) {
    globalData.motorPins   = {.gpioDirectionA = RobotEnv::GPIO_DIRECTION_A,
                              .gpioDirectionB = RobotEnv::GPIO_DIRECTION_B,
                              .gpioPWMA       = RobotEnv::GPIO_PWM_A,
                              .gpioPWMB       = RobotEnv::GPIO_PWM_B};
    globalData.motorDriver = new MotorDriver(globalData.motorPins);
  }
  if(globalData.irSensorDriver == nullptr) {
    IRSensorParamSchema irSensorParam = {
        .pins             = {.gpioMultiplexerDigitalAddress =
                                 RobotEnv::GPIO_MULTIPLEXER_DIGITAL_ADDRESS,
                             .gpioMultiplexerAnalogInput =
                                 RobotEnv::GPIO_MULTIPLEXER_ANALOG_INPUT},
        .lineSensorsCount = 12,
        .lineSensorsMultiplexerIndex =
            RobotEnv::GPIO_MULTIPLEXER_LINE_SENSORS_INDEX,
        .sideSensorsCount = 4,
        .sideSensorsMultiplexerIndex =
            RobotEnv::GPIO_MULTIPLEXER_SIDE_SENSORS_INDEX,
        .multiplexerPinCount = 4,
    };
    globalData.irSensorDriver = new IRSensorDriver(irSensorParam);
  }
  if(globalData.encoderLeftDriver == nullptr) {
    globalData.encoderLeftDriver = new EncoderDriver();
    globalData.encoderLeftDriver->attachFullQuad(RobotEnv::GPIO_ENCODER_LEFT_A,
                                                 RobotEnv::GPIO_ENCODER_LEFT_B);
  }
  if(globalData.encoderRightDriver == nullptr) {
    globalData.encoderRightDriver = new EncoderDriver();
    globalData.encoderRightDriver->attachFullQuad(
        RobotEnv::GPIO_ENCODER_RIGHT_A, RobotEnv::GPIO_ENCODER_RIGHT_B);
  }
  if(globalData.vacuumDriver == nullptr) {
    globalData.vacuumPins   = {.gpioPWM = RobotEnv::GPIO_PWM_VACUUM};
    globalData.vacuumDriver = new VacuumDriver(globalData.vacuumPins);
  }
  if(globalData.ledRgbDriver == nullptr) {
    globalData.ledRgbPins   = {.gpioData = (gpio_num_t)RobotEnv::GPIO_LED_DEBUG,
                               .numLeds  = RobotEnv::NUM_LEDS_DEBUG};
    globalData.ledRgbDriver = new LedRgbDriver(globalData.ledRgbPins);
  }


  PathControllerParamSchema pathControllerParam = {
      .constants      = {.kP = 0.017F, .kI = 0.00F, .kD = 0.068F},
      .sensorQuantity = 12,
      .sensorValues   = lineSensorValues,
      .maxAngle       = 45.0F, // Ângulo máximo de 45 graus
      .radiusSensor   = 100, // Raio dos sensores em mm
      .sensorToCenter = 50, // Distância do sensor ao centro em mm
  };
  PathController *pathController = new PathController(pathControllerParam);

  bool lastLeftReadIsOnMark  = false;
  bool lastRightReadIsOnMark = false;

  int32_t  sideSensorReadCount  = 0;
  uint32_t sideSensorAverage[4] = {0, 0, 0, 0};

  uint32_t mapPointIndex = 0;

  bool alternateLedColorFlag = false;

  globalData.ledRgbDriver->setColor(0, LED_COLOR_YELLOW);
  globalData.ledRgbDriver->refresh();
  ESP_LOGI("MainTask", "Calibrando os sensores...");
  for(int i = 0; i < 50; i++) {
    globalData.irSensorDriver->calibrate();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
  QTRSensors::CalibrationData calibrationData =
      globalData.irSensorDriver->getCalibrationData();
  ESP_LOGI("MainTask", "Calibration data: %d", calibrationData.initialized);
  ESP_LOGI("MainTask", "Calibration data: %d", calibrationData.minimum);
  ESP_LOGI("MainTask", "Calibration data: %d", calibrationData.maximum);
  if(!calibrationData.initialized ||
     (calibrationData.maximum - calibrationData.minimum < 50)) {
    globalData.isProperlyCalibrated = false;
    ESP_LOGI("MainTask", "Sensores não calibrados");
    pushMessageToQueue("Error: Sensores calibrados incorretamente (range: %d, "
                       "%d, difference: %d, initialized: %d)",
                       calibrationData.minimum, calibrationData.maximum,
                       calibrationData.maximum - calibrationData.minimum,
                       calibrationData.initialized);
  } else {
    globalData.isProperlyCalibrated = true;
    ESP_LOGI("MainTask", "Sensores calibrados");
    pushMessageToQueue("Sensores calibrados corretamente (range: %d, "
                       "%d, difference: %d, initialized: %d)",
                       calibrationData.minimum, calibrationData.maximum,
                       calibrationData.maximum - calibrationData.minimum,
                       calibrationData.initialized);
  }
  globalData.ledRgbDriver->setColor(0, LED_COLOR_GREEN);
  globalData.ledRgbDriver->refresh();

  RobotStateMachine::toIdle(globalData.motorDriver, globalData.vacuumDriver);
  ESP_LOGI("MainTask", "Escolha entre modo de mapeamento ou modo de corrida");

  for(;;) {
    if(RobotStateMachine::get() == RobotState::RUNNING) {
      for(;;) {
        int32_t encoderMilimetersAverage =
            (((globalData.encoderLeftDriver->getCount() +
               globalData.encoderRightDriver->getCount()) /
              2) *
             RobotEnv::WHEEL_CIRCUMFERENCE /
             RobotEnv::ENCODER_PULSES_PER_ROTATION) *
            -1;

        // Condição de parada
        if(encoderMilimetersAverage > finishLineCount ||
           RobotStateMachine::get() != RobotState::RUNNING) {
          globalData.motorDriver->pwmOutput(0, 0);
          globalData.vacuumDriver->pwmOutput(0);

          RobotStateMachine::toIdle(globalData.motorDriver,
                                    globalData.vacuumDriver);
          globalData.ledRgbDriver->setColor(0, LED_COLOR_RED);
          globalData.ledRgbDriver->refresh();
          break; // Back to outer loop (IDLE branch)
        }

        globalData.irSensorDriver->readCalibrated(lineSensorValues,
                                                  sideSensorValues);

        // sideSensorReadCount++;
        // for(int i = 0; i < 4; i++) {
        //   sideSensorAverage[i] += sideSensorValues[i];
        // }

        // if(sideSensorReadCount >= RobotEnv::SIDE_SENSOR_READ_AVERAGE_COUNT) {
        //   for(int i = 0; i < 4; i++) {
        //     sideSensorAverage[i] /= RobotEnv::SIDE_SENSOR_READ_AVERAGE_COUNT;
        //   }
        //   sideSensorReadCount = 0;

        //   bool leftIsOnMark =
        //       sideSensorAverage[0] < 200 || sideSensorAverage[1] < 200;
        //   bool rightIsOnMark =
        //       sideSensorAverage[2] < 200 || sideSensorAverage[3] < 200;
        //   if(!leftIsOnMark && !rightIsOnMark) {
        //     lastLeftReadIsOnMark  = false;
        //     lastRightReadIsOnMark = false;
        //   } else if(!leftIsOnMark && rightIsOnMark) {
        //     if(!lastRightReadIsOnMark) {
        //       lastLeftReadIsOnMark  = false;
        //       lastRightReadIsOnMark = true;
        //     }
        //   } else if(leftIsOnMark && !rightIsOnMark) {
        //     if(!lastLeftReadIsOnMark) {
        //       globalData.markCount.store(
        //           globalData.markCount.load(std::memory_order_relaxed) + 1,
        //           std::memory_order_relaxed);

        //       lastLeftReadIsOnMark  = true;
        //       lastRightReadIsOnMark = false;
        //     }
        //   } else {
        //     lastLeftReadIsOnMark  = true;
        //     lastRightReadIsOnMark = true;
        //   }

        //   for(int i = 0; i < 4; i++) {
        //     sideSensorAverage[i] = 0;
        //   }
        // }

        if(globalData.mapData[mapPointIndex].encoderMilimeters >
               encoderMilimetersAverage &&
           (mapPointIndex + 1) < globalData.mapData.size()) {
          mapPointIndex++;
          if(alternateLedColorFlag) {
            globalData.ledRgbDriver->setColor(0, LED_COLOR_ORANGE);
            alternateLedColorFlag = false;
          } else {
            globalData.ledRgbDriver->setColor(0, LED_COLOR_CYAN);
            alternateLedColorFlag = true;
          }
          globalData.ledRgbDriver->refresh();
        }

        float pathPID = pathController->getPID();

        globalData.motorDriver->pwmOutput(
            globalData.mapData[mapPointIndex].baseMotorPWM + pathPID,
            globalData.mapData[mapPointIndex].baseMotorPWM - pathPID);

        // globalData.vacuumDriver->pwmOutput(
        //     globalData.mapData[mapPointIndex].baseVacuumPWM);
        globalData.vacuumDriver->pwmOutput(
            globalData.parametersConfig.vacuumPWM);

        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
    } else if(RobotStateMachine::get() == RobotState::MAPPING) {
      for(;;) {
        int32_t encoderMilimetersAverage =
            (((globalData.encoderLeftDriver->getCount() +
               globalData.encoderRightDriver->getCount()) /
              2) *
             RobotEnv::WHEEL_CIRCUMFERENCE /
             RobotEnv::ENCODER_PULSES_PER_ROTATION) *
            -1;

        // Condição de parada
        if(RobotStateMachine::get() != RobotState::MAPPING) {
          globalData.motorDriver->pwmOutput(0, 0);
          globalData.vacuumDriver->pwmOutput(0);

          globalData.mapData.push_back(
              {.encoderMilimeters = encoderMilimetersAverage,
               .baseMotorPWM      = RobotEnv::MAPPING_MOTOR_PWM,
               .baseVacuumPWM     = RobotEnv::BASE_VACUUM_PWM,
               .markType          = MapPoint::MarkType::STOP_COMMAND_MARK});

          RobotStateMachine::toIdle(globalData.motorDriver,
                                    globalData.vacuumDriver);
          break; // Back to outer loop (IDLE branch)
        }

        globalData.irSensorDriver->readCalibrated(lineSensorValues,
                                                  sideSensorValues);

        sideSensorReadCount++;
        for(int i = 0; i < 4; i++) {
          sideSensorAverage[i] += sideSensorValues[i];
        }

        if(sideSensorReadCount >= RobotEnv::SIDE_SENSOR_READ_AVERAGE_COUNT) {
          for(int i = 0; i < 4; i++) {
            sideSensorAverage[i] /= RobotEnv::SIDE_SENSOR_READ_AVERAGE_COUNT;
          }
          sideSensorReadCount = 0;

          bool leftIsOnMark =
              sideSensorAverage[0] < 200 || sideSensorAverage[1] < 200;
          bool rightIsOnMark =
              sideSensorAverage[2] < 200 || sideSensorAverage[3] < 200;
          if(!leftIsOnMark && !rightIsOnMark) {
            lastLeftReadIsOnMark  = false;
            lastRightReadIsOnMark = false;
          } else if(!leftIsOnMark && rightIsOnMark) {
            if(!lastRightReadIsOnMark) {
              lastLeftReadIsOnMark  = false;
              lastRightReadIsOnMark = true;

              ESP_LOGI("MainTask", "Mark found on the right side");
              globalData.mapData.push_back(
                  {.encoderMilimeters = encoderMilimetersAverage,
                   .baseMotorPWM      = RobotEnv::MAPPING_MOTOR_PWM,
                   .baseVacuumPWM     = RobotEnv::BASE_VACUUM_PWM,
                   .markType          = MapPoint::MarkType::RIGHT_MARK});
              if(alternateLedColorFlag) {
                globalData.ledRgbDriver->setColor(0, LED_COLOR_GREEN);
                alternateLedColorFlag = false;
              } else {
                globalData.ledRgbDriver->setColor(0, LED_COLOR_RED);
                alternateLedColorFlag = true;
              }
              globalData.ledRgbDriver->refresh();
            }
          } else if(leftIsOnMark && !rightIsOnMark) {
            if(!lastLeftReadIsOnMark) {
              globalData.markCount.store(
                  globalData.markCount.load(std::memory_order_relaxed) + 1,
                  std::memory_order_relaxed);

              globalData.mapData.push_back(
                  {.encoderMilimeters = encoderMilimetersAverage,
                   .baseMotorPWM      = RobotEnv::MAPPING_MOTOR_PWM,
                   .baseVacuumPWM     = RobotEnv::BASE_VACUUM_PWM,
                   .markType          = MapPoint::MarkType::LEFT_MARK});

              lastLeftReadIsOnMark  = true;
              lastRightReadIsOnMark = false;
              if(alternateLedColorFlag) {
                globalData.ledRgbDriver->setColor(0, LED_COLOR_GREEN);
                alternateLedColorFlag = false;
              } else {
                globalData.ledRgbDriver->setColor(0, LED_COLOR_RED);
                alternateLedColorFlag = true;
              }
              globalData.ledRgbDriver->refresh();
            }
          } else {
            lastLeftReadIsOnMark  = true;
            lastRightReadIsOnMark = true;
          }

          for(int i = 0; i < 4; i++) {
            sideSensorAverage[i] = 0;
          }
        }

        float pathPID = pathController->getPID();

        globalData.motorDriver->pwmOutput(RobotEnv::MAPPING_MOTOR_PWM + pathPID,
                                          RobotEnv::MAPPING_MOTOR_PWM -
                                              pathPID);

        // printf("\033[2J\033[H");
        // for(int i = 0; i < 16; i++) {
        //   printf("%4d ", lineSensorValues[i]);
        // }
        // printf("L: ");
        // for(int i = 0; i < 12; i++) {
        //   printf("%4d ", lineSensorValues[i]);
        // }
        // printf("S: ");
        // for(int i = 0; i < 4; i++) {
        //   printf("%4d ", sideSensorValues[i]);
        // }
        // printf("\n");
        // printf("Base Motor PWM: %ld\n",
        //        globalData.mapData[mapPointIndex].baseMotorPWM);
        // printf("Path PID: %f\n", pathPID);
        // printf("Mark Count: %ld\n",
        //        globalData.markCount.load(std::memory_order_relaxed));
        // printf("Encoder Left: %ld\n", encoderLeftDriver->getCount());
        // printf("Encoder Right: %ld\n", encoderRightDriver->getCount());

        vTaskDelay(1 / portTICK_PERIOD_MS);
      }
    } else {
      // globalData.irSensorDriver->readCalibrated(lineSensorValues,
      //                                           sideSensorValues);
      // printf("\033[2J\033[H");
      // for(int i = 0; i < 16; i++) {
      //   printf("%4d ", lineSensorValues[i]);
      // }

      // IDLE or other state: keep task alive and re-check state periodically
      if(globalData.isProperlyCalibrated) {
        if(alternateLedColorFlag) {
          globalData.ledRgbDriver->setColor(0, LED_COLOR_PURPLE, 0.75f);
          alternateLedColorFlag = false;
        } else {
          globalData.ledRgbDriver->setColor(0, LED_COLOR_WHITE, 0.75f);
          alternateLedColorFlag = true;
        }
        globalData.ledRgbDriver->setColor(1, LED_COLOR_BLUE);
        globalData.ledRgbDriver->setColor(2, LED_COLOR_BLUE);
        globalData.ledRgbDriver->setColor(3, LED_COLOR_BLUE);
      } else {
        globalData.ledRgbDriver->setColor(0, LED_COLOR_RED);
        globalData.ledRgbDriver->setColor(1, LED_COLOR_RED);
        globalData.ledRgbDriver->setColor(2, LED_COLOR_RED);
        globalData.ledRgbDriver->setColor(3, LED_COLOR_RED);
      }

      globalData.ledRgbDriver->refresh();
      globalData.motorDriver->pwmOutput(0, 0);
      globalData.vacuumDriver->pwmOutput(0);
      vTaskDelay(100 / portTICK_PERIOD_MS);
    }
  }
}

#endif // MAIN_TASK_HPP
