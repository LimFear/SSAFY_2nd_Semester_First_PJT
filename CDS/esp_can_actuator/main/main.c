#include "driver/gpio.h"
#include "driver/twai.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdio.h>

static const char *TAG = "PHASE_E_UNIFIED";

// ==========================================
// 노드 역할 설정
// ==========================================
#define ROLE_SENSOR 0
#define ROLE_ACTUATOR 1

// 아래 0을 1로 바꾸면 액츄에이터 모드가 됩니다.
#define CURRENT_ROLE 1
// ==========================================

// GPIO 설정
#define TX_GPIO_NUM 33
#define RX_GPIO_NUM 32
#define SENSOR_ADC_CHANNEL ADC_CHANNEL_6
#define ACTU_R_GPIO 4
#define ACTU_G_GPIO 5
#define ACTU_B_GPIO 18

void app_main(void) {
  // 1. CAN 드라이버 설정
  twai_general_config_t g_config =
      TWAI_GENERAL_CONFIG_DEFAULT(TX_GPIO_NUM, RX_GPIO_NUM, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
  ESP_ERROR_CHECK(twai_start());

  if (CURRENT_ROLE == ROLE_SENSOR) {
    ESP_LOGI(TAG, "Mode: SENSOR NODE (0x101)");

    adc_oneshot_unit_handle_t adc1_handle;
    adc_oneshot_unit_init_cfg_t init_config1 = {.unit_id = ADC_UNIT_1};
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));

    adc_oneshot_chan_cfg_t config = {.bitwidth = ADC_BITWIDTH_DEFAULT,
                                     .atten = ADC_ATTEN_DB_12};
    ESP_ERROR_CHECK(
        adc_oneshot_config_channel(adc1_handle, SENSOR_ADC_CHANNEL, &config));

    int adc_raw;
    while (1) {
      ESP_ERROR_CHECK(
          adc_oneshot_read(adc1_handle, SENSOR_ADC_CHANNEL, &adc_raw));
      ESP_LOGI(TAG, "Light Sensor Raw: %d", adc_raw);

      twai_message_t tx_msg = {.identifier = 0x101,
                               .data_length_code = 8,
                               .data = {(uint8_t)(adc_raw >> 8),
                                        (uint8_t)(adc_raw & 0xFF), 0, 0, 0, 0,
                                        0, 0}};
      twai_transmit(&tx_msg, pdMS_TO_TICKS(100));
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  } else {
    ESP_LOGI(TAG, "Mode: ACTUATOR NODE (0x201)");

    gpio_reset_pin(ACTU_R_GPIO);
    gpio_set_direction(ACTU_R_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ACTU_G_GPIO);
    gpio_set_direction(ACTU_G_GPIO, GPIO_MODE_OUTPUT);
    gpio_reset_pin(ACTU_B_GPIO);
    gpio_set_direction(ACTU_B_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
      twai_message_t rx_msg;
      if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
        if (rx_msg.identifier == 0x201) {
          uint8_t r = rx_msg.data[0];
          uint8_t g = rx_msg.data[1];
          uint8_t b = rx_msg.data[2];

          gpio_set_level(ACTU_R_GPIO, r ? 1 : 0);
          gpio_set_level(ACTU_G_GPIO, g ? 1 : 0);
          gpio_set_level(ACTU_B_GPIO, b ? 1 : 0);

          ESP_LOGI(TAG, "RGB LED -> R:%d, G:%d, B:%d", r, g, b);
        }
      }
    }
  }
}
