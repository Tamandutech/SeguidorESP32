#ifndef LEDRGB_DRIVER_HPP
#define LEDRGB_DRIVER_HPP

#include <cstring>

#include "esp_attr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/rmt_tx.h"
#include "freertos/FreeRTOS.h"

#include "LedStripEncoder/led_strip_encoder.h"

#define RMT_LED_STRIP_RESOLUTION_HZ 10000000 // 10 MHz, 1 tick = 0.1 us

struct LedRgbPins {
  gpio_num_t gpioData;
  uint8_t    numLeds;
};

enum LedColor {
  LED_COLOR_BLACK   = 0x000000,
  LED_COLOR_RED     = 0xFF0000,
  LED_COLOR_GREEN   = 0x00FF00,
  LED_COLOR_BLUE    = 0x0000FF,
  LED_COLOR_YELLOW  = 0xFFFF00,
  LED_COLOR_CYAN    = 0x00FFFF,
  LED_COLOR_MAGENTA = 0xFF00FF,
  LED_COLOR_WHITE   = 0xFFFFFF,
  LED_COLOR_ORANGE  = 0xFF7F00,
  LED_COLOR_PURPLE  = 0x7F007F,
};

struct LedRgbColor {
  uint8_t blue  = 0;
  uint8_t red   = 0;
  uint8_t green = 0;
};

class LedRgbDriver {
public:
  LedRgbDriver(LedRgbPins pins);
  ~LedRgbDriver();

  void setColor(uint8_t ledIndex, LedColor color, float brightness = 1.0f);
  void setColorRgb(uint8_t ledIndex, uint8_t r, uint8_t g, uint8_t b);
  void refresh();

private:
  const char *tag_ = "LedRgbDriver";
  LedRgbPins  pins_;

  rmt_channel_handle_t  ledChan_    = nullptr;
  rmt_encoder_handle_t  ledEncoder_ = nullptr;
  rmt_transmit_config_t txConfig_{};
  uint8_t              *ledStripPixels_ = nullptr;

  void stripInit();
  void colorToRgb(LedColor color, uint8_t *r, uint8_t *g, uint8_t *b);
};

LedRgbDriver::LedRgbDriver(LedRgbPins pins) : pins_(pins) {
  ledStripPixels_ = (uint8_t *)calloc(pins_.numLeds * 3, sizeof(uint8_t));
  if(ledStripPixels_ == nullptr) {
    ESP_LOGE(tag_, "Failed to allocate pixel buffer");
    return;
  }
  stripInit();
  memset(ledStripPixels_, 0, pins_.numLeds * 3);
  ESP_LOGD(tag_, "LedRgbDriver init: GPIO %d, %u LEDs", pins_.gpioData,
           pins_.numLeds);
}

LedRgbDriver::~LedRgbDriver() {
  if(ledChan_ != nullptr) {
    rmt_disable(ledChan_);
    rmt_del_channel(ledChan_);
    ledChan_ = nullptr;
  }
  if(ledEncoder_ != nullptr) {
    rmt_del_encoder(ledEncoder_);
    ledEncoder_ = nullptr;
  }
  free(ledStripPixels_);
  ledStripPixels_ = nullptr;
}

void LedRgbDriver::stripInit() {
  rmt_tx_channel_config_t txChanConfig = {
      .gpio_num          = pins_.gpioData,
      .clk_src           = RMT_CLK_SRC_DEFAULT,
      .resolution_hz     = RMT_LED_STRIP_RESOLUTION_HZ,
      .mem_block_symbols = 64,
      .trans_queue_depth = 4,
  };
  esp_err_t err = rmt_new_tx_channel(&txChanConfig, &ledChan_);
  if(err != ESP_OK) {
    ESP_LOGE(tag_, "rmt_new_tx_channel failed: %s", esp_err_to_name(err));
    return;
  }

  led_strip_encoder_config_t encoderConfig = {
      .resolution = RMT_LED_STRIP_RESOLUTION_HZ,
  };
  err = rmt_new_led_strip_encoder(&encoderConfig, &ledEncoder_);
  if(err != ESP_OK) {
    ESP_LOGE(tag_, "rmt_new_led_strip_encoder failed: %s",
             esp_err_to_name(err));
    rmt_del_channel(ledChan_);
    ledChan_ = nullptr;
    return;
  }

  err = rmt_enable(ledChan_);
  if(err != ESP_OK) {
    ESP_LOGE(tag_, "rmt_enable failed: %s", esp_err_to_name(err));
    rmt_del_encoder(ledEncoder_);
    rmt_del_channel(ledChan_);
    ledEncoder_ = nullptr;
    ledChan_    = nullptr;
    return;
  }

  txConfig_.loop_count = 0;
}

void LedRgbDriver::colorToRgb(LedColor color, uint8_t *r, uint8_t *g,
                              uint8_t *b) {
  const uint8_t *bytes = (const uint8_t *)&color;
  *r                   = bytes[0];
  *g                   = bytes[1];
  *b                   = bytes[2];
}

void LedRgbDriver::setColor(uint8_t ledIndex, LedColor color,
                            float brightness) {
  if(ledStripPixels_ == nullptr || ledIndex >= pins_.numLeds) {
    return;
  }
  if(brightness > 1.0f) {
    brightness = 1.0f;
  } else if(brightness < 0.0f) {
    brightness = 0.0f;
  }
  uint8_t r, g, b;
  colorToRgb(color, &r, &g, &b);
  r = (uint8_t)(brightness * r);
  g = (uint8_t)(brightness * g);
  b = (uint8_t)(brightness * b);
  // WS2812 order: G, B, R
  ledStripPixels_[ledIndex * 3 + 0] = g;
  ledStripPixels_[ledIndex * 3 + 1] = b;
  ledStripPixels_[ledIndex * 3 + 2] = r;
}

void LedRgbDriver::setColorRgb(uint8_t ledIndex, uint8_t r, uint8_t g,
                               uint8_t b) {
  if(ledStripPixels_ == nullptr || ledIndex >= pins_.numLeds) {
    return;
  }
  ledStripPixels_[ledIndex * 3 + 0] = g;
  ledStripPixels_[ledIndex * 3 + 1] = b;
  ledStripPixels_[ledIndex * 3 + 2] = r;
}

void LedRgbDriver::refresh() {
  if(ledChan_ == nullptr || ledEncoder_ == nullptr ||
     ledStripPixels_ == nullptr) {
    return;
  }
  size_t    size = pins_.numLeds * 3;
  esp_err_t err =
      rmt_transmit(ledChan_, ledEncoder_, ledStripPixels_, size, &txConfig_);
  if(err == ESP_OK) {
    rmt_tx_wait_all_done(ledChan_, portMAX_DELAY);
  }
}

#endif // LEDRGB_DRIVER_HPP
