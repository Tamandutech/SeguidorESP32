#ifndef ENCODER_DRIVER_HPP
#define ENCODER_DRIVER_HPP

#include "driver/pulse_cnt.h"
#include "esp_attr.h"
#include "esp_log.h"
#include <driver/gpio.h>
#include <stdbool.h>

enum EncoderType { single, half, full };

class EncoderDriver {
public:
  EncoderDriver();
  ~EncoderDriver();

  void attachHalfQuad(int aPintNumber, int bPinNumber);
  void attachFullQuad(int aPintNumber, int bPinNumber);
  void attachSingleEdge(int aPintNumber, int bPinNumber);
  // void attachHalfQuad(int aPintNumber, int bPinNumber);
  int32_t getCount();
  int32_t getCountRaw();
  int32_t clearCount();
  int32_t pauseCount();
  int32_t resumeCount();

  bool               isAttached() { return attached; }
  void               setCount(int32_t value);
  static bool        attachedInterrupt;
  gpio_num_t         aPinNumber;
  gpio_num_t         bPinNumber;
  pcnt_unit_config_t unit;
  pcnt_unit_handle_t pcnt_unit  = NULL;
  bool               fullQuad   = false;
  int                countsMode = 2;
  volatile int32_t   count      = 0;
  pcnt_chan_config_t channA;
  pcnt_chan_config_t channB;
  static bool        useInternalWeakPullResistors;

private:
  void attach(int aPintNumber, int bPinNumber, enum EncoderType et);
  bool attached = false;
  bool direction;
  bool working;
};

bool EncoderDriver::useInternalWeakPullResistors = true;

EncoderDriver::EncoderDriver() {
  attached   = false;
  aPinNumber = (gpio_num_t)0;
  bPinNumber = (gpio_num_t)0;
  working    = false;
  direction  = false;
}

EncoderDriver::~EncoderDriver() {
  // TODO Auto-generated destructor stub
}

void EncoderDriver::attach(int a, int b, enum EncoderType et) {
  if(attached) {
    ESP_LOGE("EncoderDriver", "All ready attached, FAIL!");
    return;
  }

  // Set data now that pin attach checks are done
  fullQuad = et != single;
  unit     = {
          .low_limit  = INT16_MIN,
          .high_limit = INT16_MAX,
  };
  unit.flags.accum_count = 1;
  this->aPinNumber       = (gpio_num_t)a;
  this->bPinNumber       = (gpio_num_t)b;

  ESP_ERROR_CHECK(pcnt_new_unit(&unit, &pcnt_unit));

  ESP_LOGI("EncoderDriver", "set glitch filter");
  pcnt_glitch_filter_config_t filter_config = {
      .max_glitch_ns = 1000,
  };
  ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

  // Set up the IO state of hte pin
  gpio_set_direction(aPinNumber, GPIO_MODE_INPUT);
  gpio_set_direction(bPinNumber, GPIO_MODE_INPUT);
  if(useInternalWeakPullResistors) {
    gpio_pulldown_en(aPinNumber);
    gpio_pulldown_en(bPinNumber);
  }

  ESP_LOGI("EncoderDriver", "install pcnt channels");
  channA = {
      .edge_gpio_num  = this->aPinNumber,
      .level_gpio_num = this->bPinNumber,
  };
  pcnt_channel_handle_t pcnt_chan_a = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &channA, &pcnt_chan_a));

  pcnt_chan_config_t channB = {
      .edge_gpio_num  = this->bPinNumber,
      .level_gpio_num = this->aPinNumber,
  };
  pcnt_channel_handle_t pcnt_chan_b = NULL;
  ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &channB, &pcnt_chan_b));

  // Set up encoder PCNT configuration
  ESP_ERROR_CHECK(
      pcnt_channel_set_edge_action(pcnt_chan_a,
                                   fullQuad ? PCNT_CHANNEL_EDGE_ACTION_DECREASE
                                            : PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                   PCNT_CHANNEL_EDGE_ACTION_INCREASE));
  ESP_ERROR_CHECK(pcnt_channel_set_level_action(
      pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_INVERSE,
      PCNT_CHANNEL_LEVEL_ACTION_KEEP));

  if(et == full) {
    // set up second channel for full quad
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(
        pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_DECREASE,
        PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP,
        PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
  } else { // make sure channel 1 is not set when not full quad
    ESP_ERROR_CHECK(
        pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_HOLD,
                                     PCNT_CHANNEL_EDGE_ACTION_HOLD));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(
        pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_HOLD,
        PCNT_CHANNEL_LEVEL_ACTION_HOLD));
  }

  /* Enable events on  maximum and minimum limit values */
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, INT16_MAX));
  ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, INT16_MIN));


  ESP_LOGI("EncoderDriver", "enable pcnt unit");
  ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
  ESP_LOGI("EncoderDriver", "clear pcnt unit");
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  ESP_LOGI("EncoderDriver", "start pcnt unit");
  ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
}

void EncoderDriver::attachHalfQuad(int aPintNumber, int bPinNumber) {
  attach(aPintNumber, bPinNumber, half);
}
void EncoderDriver::attachSingleEdge(int aPintNumber, int bPinNumber) {
  attach(aPintNumber, bPinNumber, single);
}
void EncoderDriver::attachFullQuad(int aPintNumber, int bPinNumber) {
  attach(aPintNumber, bPinNumber, full);
}

void EncoderDriver::setCount(int32_t value) { count = value - getCountRaw(); }
int32_t EncoderDriver::getCountRaw() {
  int c;
  ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &c));
  return c;
}
int32_t EncoderDriver::getCount() { return getCountRaw() + count; }

int32_t EncoderDriver::clearCount() {
  count = 0;
  ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
  return ESP_OK;
}

int32_t EncoderDriver::pauseCount() { return pcnt_unit_stop(pcnt_unit); }

int32_t EncoderDriver::resumeCount() { return pcnt_unit_start(pcnt_unit); }

#endif // ENCODER_DRIVER_HPP