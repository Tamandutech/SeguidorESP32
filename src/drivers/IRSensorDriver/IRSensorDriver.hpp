#ifndef IRSENSOR_DRIVER_HPP
#define IRSENSOR_DRIVER_HPP

#include "esp_attr.h"
#include "esp_system.h"
#include <bitset>

#include "driver/gpio.h"

#include "QTRSensors/QTRSensors.h"

struct IRSensorPins {
  // defina para cada controlador
  const uint8_t *gpioMultiplexerDigitalAddress;
  const uint8_t  gpioMultiplexerAnalogInput;
};

struct IRSensorParamSchema {
  const IRSensorPins pins;
  const uint8_t      lineSensorsCount;
  const uint8_t     *lineSensorsMultiplexerIndex;
  const uint8_t      sideSensorsCount;
  const uint8_t     *sideSensorsMultiplexerIndex;
  const uint8_t      multiplexerPinCount;
};

class IRSensorDriver {
public:
  IRSensorDriver(IRSensorParamSchema param);
  ~IRSensorDriver();

  void read(uint16_t *sensor_values);
  void readCalibrated(uint16_t *line_sensor_values,
                      uint16_t *side_sensor_values);
  void calibrate();

  /// Underlying QTR driver. Use \c calibrationOn / \c calibrationOff (public on
  /// \c QTRSensors) to read or write per-channel min/max after arrays exist
  /// (e.g. after \c calibrate() or \c setCalibrationOn()).
  QTRSensors &qtrSensors() { return sensorsArray_; }

  /// Total QTR channels (line + side), matches \c QTRSensors::_sensorCount.
  uint8_t getSensorCount() const {
    return lineSensorsCount_ + sideSensorsCount_;
  }

private:
  IRSensorPins   pins_;
  const uint8_t  lineSensorsCount_;
  const uint8_t *lineSensorsMultiplexerIndex_;
  const uint8_t  sideSensorsCount_;
  const uint8_t *sideSensorsMultiplexerIndex_;
  uint8_t        multiplexerPinCount_;

  QTRSensors sensorsArray_;
  uint16_t  *sensorValues_;

  void setMultiplexerDigitalAddress(std::bitset<4> address);
};

IRSensorDriver::IRSensorDriver(IRSensorParamSchema param)
    : pins_(param.pins), lineSensorsCount_(param.lineSensorsCount),
      lineSensorsMultiplexerIndex_(param.lineSensorsMultiplexerIndex),
      sideSensorsCount_(param.sideSensorsCount),
      sideSensorsMultiplexerIndex_(param.sideSensorsMultiplexerIndex),
      multiplexerPinCount_(param.multiplexerPinCount),
      sensorValues_((uint16_t *)calloc(lineSensorsCount_ + sideSensorsCount_,
                                       sizeof(uint16_t))) {

  // Configure GPIO_MULTIPLEXER_DIGITAL_ADDRESS pins as outputs
  gpio_config_t io_conf = {};
  io_conf.intr_type     = GPIO_INTR_DISABLE;
  io_conf.mode          = GPIO_MODE_OUTPUT;
  io_conf.pin_bit_mask  = 0;
  io_conf.pull_down_en  = GPIO_PULLDOWN_DISABLE;
  io_conf.pull_up_en    = GPIO_PULLUP_DISABLE;

  // Set pin bit mask for all multiplexer digital address pins
  for(int i = 0; i < multiplexerPinCount_; i++) {
    io_conf.pin_bit_mask |= (1ULL << pins_.gpioMultiplexerDigitalAddress[i]);
  }

  gpio_config(&io_conf);

  sensorsArray_.setTypeMultiplexer();
  sensorsArray_.setSensorPins(param.pins.gpioMultiplexerAnalogInput,
                              param.lineSensorsCount + param.sideSensorsCount,
                              param.pins.gpioMultiplexerDigitalAddress,
                              param.multiplexerPinCount);
}

IRSensorDriver::~IRSensorDriver() { free(sensorValues_); }

void IRSensorDriver::setMultiplexerDigitalAddress(std::bitset<4> address) {
  // Função que controla qual sensor está conectado ao ESP32
  // Recebe um vetor com a sequencia de 4 bits de 0000 a 1111 e ajusta o nível
  // lógico das portas digitais de acordo 0010
  for(int i = 0; i < address.size(); i++) {
    if(address[i]) {
      gpio_set_level((gpio_num_t)pins_.gpioMultiplexerDigitalAddress[i], 1);
    } // se bit=1, nível lógico alto
    else {
      gpio_set_level((gpio_num_t)pins_.gpioMultiplexerDigitalAddress[i], 0);
    } // se bit=0, nível lógico baixo
  }
}

void IRSensorDriver::read(uint16_t *sensor_values) {
  sensorsArray_.read(sensor_values);
}

void IRSensorDriver::calibrate() { sensorsArray_.calibrate(); }

void IRSensorDriver::readCalibrated(uint16_t *line_sensor_values,
                                    uint16_t *side_sensor_values) {
  sensorsArray_.readCalibrated(sensorValues_);
  for(int i = 0; i < (lineSensorsCount_); i++) {
    line_sensor_values[i] = sensorValues_[lineSensorsMultiplexerIndex_[i]];
  }
  for(int i = 0; i < (sideSensorsCount_); i++) {
    side_sensor_values[i] = sensorValues_[sideSensorsMultiplexerIndex_[i]];
  }
}

#endif // IRSENSOR_DRIVER_HPP