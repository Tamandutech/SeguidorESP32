#include "system.hpp"

#include <cstdio>

#include "esp_log.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

#include "context/GlobalData.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "tasks/cli/cli.hpp"
#include "tasks/cli/wire_protocol.hpp"
#include "tasks/StateMachineTask.hpp"

namespace cli_system {

namespace {
const char *TAG = "cli_system";

adc_oneshot_unit_handle_t getBatteryAdcHandle() {
  static adc_oneshot_unit_handle_t adc2_handle = nullptr;
  static bool                      initialized = false;

  if(!initialized) {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id  = ADC_UNIT_2,
        .clk_src  = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config, &adc2_handle);
    if(ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to initialize ADC2 for battery voltage");
      return nullptr;
    }

    adc_oneshot_chan_cfg_t config = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ret = adc_oneshot_config_channel(adc2_handle, ADC_CHANNEL_7, &config);
    if(ret != ESP_OK) {
      ESP_LOGE(TAG, "Failed to configure ADC2 channel for battery voltage");
      adc_oneshot_del_unit(adc2_handle);
      adc2_handle = nullptr;
      return nullptr;
    }

    initialized = true;
  }

  return adc2_handle;
}

} // namespace

int wirePause() {
  StateMachineTask *sm = cli_active_state_machine();
  if(sm == nullptr) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  Event ev{EventType::STOP};
  (void)sm->postEvent(ev, pdMS_TO_TICKS(10));
  wire::emitSingleResponse("pause", {"ok"});
  return CLI_SUCCESS;
}

int wireResume() {
  StateMachineTask *sm = cli_active_state_machine();
  if(sm == nullptr) {
    return CLI_ERROR_COMMAND_NOT_FOUND;
  }
  Event ev{};
  if(globalData.parametersConfig.runOnMappingMode) {
    ev.type = EventType::MAP;
  } else {
    ev.type = EventType::START;
  }
  (void)sm->postEvent(ev, pdMS_TO_TICKS(10));
  wire::emitSingleResponse("resume", {"ok"});
  return CLI_SUCCESS;
}

int wireBatVoltage() {
  adc_oneshot_unit_handle_t adc_handle = getBatteryAdcHandle();
  char                        mv[16];
  if(adc_handle == nullptr) {
    snprintf(mv, sizeof(mv), "%d", 0);
    wire::emitSingleResponse("bat_voltage", {mv});
    return CLI_SUCCESS;
  }

  int       adc_raw = 0;
  esp_err_t ret     = adc_oneshot_read(adc_handle, ADC_CHANNEL_7, &adc_raw);

  if(ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to read battery voltage ADC: %s",
             esp_err_to_name(ret));
    snprintf(mv, sizeof(mv), "%d", 0);
    wire::emitSingleResponse("bat_voltage", {mv});
    return CLI_SUCCESS;
  }

  uint32_t voltage_mv = (static_cast<uint32_t>(adc_raw) * 3300U) / 4095U;
  snprintf(mv, sizeof(mv), "%lu", static_cast<unsigned long>(voltage_mv));
  wire::emitSingleResponse("bat_voltage", {mv});
  return CLI_SUCCESS;
}

} // namespace cli_system
