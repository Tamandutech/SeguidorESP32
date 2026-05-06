#ifndef MOTOR_DRIVER_HPP
#define MOTOR_DRIVER_HPP

#include <cmath>

#include "esp_attr.h"
#include "esp_system.h"

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"

#include "context/RobotEnv.hpp"

#define MOTOR_LEDC_TIMER     LEDC_TIMER_1        // Timer do LEDC utilizado
#define MOTOR_LEDC_MODE      LEDC_LOW_SPEED_MODE // Modo de velocidade do LEDC
#define MOTOR_PWM_A_PIN      LEDC_CHANNEL_2      // Canal do LEDC utilizado
#define MOTOR_PWM_B_PIN      LEDC_CHANNEL_3      // Canal do LEDC utilizado
#define MOTOR_LEDC_DUTY_RES  LEDC_TIMER_13_BIT   // Resolução do PWM
#define MOTOR_LEDC_FREQUENCY 2000 // Frequência em Hertz do sinal PWM


struct MotorPins {
  uint8_t gpioDirectionA;
  uint8_t gpioDirectionB;
  uint8_t gpioPWMA;
  uint8_t gpioPWMB;
};

class MotorDriver {
public:
  MotorDriver(MotorPins pin);

  void pwmOutput(int32_t valueA, int32_t valueB);

private:
  const char   *tag = "MotorDriver";
  MotorPins     pin_;
  const int32_t maxValue;

  void initMotorPWM(gpio_num_t pin, ledc_channel_t channel);
};

void MotorDriver::initMotorPWM(gpio_num_t pin, ledc_channel_t channel) {
  // Prepara e aplica a configuração do canal do LEDC
  ledc_channel_config_t ledc_channel = {.gpio_num   = pin,
                                        .speed_mode = MOTOR_LEDC_MODE,
                                        .channel    = channel,
                                        .intr_type  = LEDC_INTR_DISABLE,
                                        .timer_sel  = MOTOR_LEDC_TIMER,
                                        .duty       = 0,
                                        .hpoint     = 0};
  ledc_channel_config(&ledc_channel);
}

MotorDriver::MotorDriver(MotorPins pin)
    : pin_(pin), maxValue(pow(2, MOTOR_LEDC_DUTY_RES) - 1) {
  gpio_config_t io_conf;
  io_conf.intr_type = (gpio_int_type_t)GPIO_INTR_DISABLE;
  io_conf.mode      = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask =
      ((1ULL << pin_.gpioDirectionA) | (1ULL << pin_.gpioDirectionB));
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en   = GPIO_PULLUP_DISABLE;
  gpio_config(&io_conf);

  // Configure LEDC timer once
  ledc_timer_config_t ledc_timer = {
      .speed_mode      = MOTOR_LEDC_MODE,
      .duty_resolution = MOTOR_LEDC_DUTY_RES,
      .timer_num       = MOTOR_LEDC_TIMER,
      .freq_hz         = MOTOR_LEDC_FREQUENCY, // Frequência de 2Khz
      .clk_cfg         = LEDC_AUTO_CLK         // Configuração da fonte de clock
  };
  ledc_timer_config(&ledc_timer);

  // Install fade function once
  ledc_fade_func_install(0);

  ESP_LOGD(tag, "Init PWM Motor 0");
  initMotorPWM((gpio_num_t)pin_.gpioPWMA, MOTOR_PWM_A_PIN);

  ESP_LOGD(tag, "Init PWM Motor 1");
  initMotorPWM((gpio_num_t)pin_.gpioPWMB, MOTOR_PWM_B_PIN);

  ESP_LOGD(tag, "PWM Initialized");
}

void MotorDriver::pwmOutput(int32_t valueA, int32_t valueB) {
  // Clamp input values to (-100, 100) range
  valueA = (valueA > 100) ? 100 : (valueA < -100) ? -100 : valueA;
  valueB = (valueB > 100) ? 100 : (valueB < -100) ? -100 : valueB;

  if(valueA > RobotEnv::MAX_MOTOR_PWM) {
    valueA = RobotEnv::MAX_MOTOR_PWM;
  } else if(valueA < -RobotEnv::MAX_MOTOR_PWM) {
    valueA = -RobotEnv::MAX_MOTOR_PWM;
  }
  if(valueB > RobotEnv::MAX_MOTOR_PWM) {
    valueB = RobotEnv::MAX_MOTOR_PWM;
  } else if(valueB < -RobotEnv::MAX_MOTOR_PWM) {
    valueB = -RobotEnv::MAX_MOTOR_PWM;
  }

  // Map from (-100, 100) to (-maxValue, maxValue)
  int32_t mappedValueA = (valueA * maxValue) / 100;
  int32_t mappedValueB = (valueB * maxValue) / 100;

  if(mappedValueA > 0) {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionA), 0);
  } else {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionA), 1);
  }
  if(mappedValueB > 0) {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionB), 0);
  } else {
    gpio_set_level(static_cast<gpio_num_t>(pin_.gpioDirectionB), 1);
  }

  // Use absolute values for PWM duty cycle
  ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_PWM_A_PIN, abs(mappedValueA));
  ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_PWM_B_PIN, abs(mappedValueB));
  ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_PWM_A_PIN);
  ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_PWM_B_PIN);
}


#endif // MOTOR_DRIVER_HPP
