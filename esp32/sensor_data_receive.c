#include <Arduino.h>
#include "driver/twai.h"

// CAN통신 핀 설정 (송신과 동일)
#define CAN_RX_GPIO_PIN GPIO_NUM_32
#define CAN_TX_GPIO_PIN GPIO_NUM_33

// CAN ID (송신과 동일)
#define CAN_DHT_ID 0x100

// CAN 기본 환경 설정
bool can_driver_init() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_PIN, CAN_RX_GPIO_PIN, TWAI_MODE_NORMAL);

    twai_timing_config_t t_config =
        TWAI_TIMING_CONFIG_500KBITS();

    twai_filter_config_t f_config =
        TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
        Serial.println("CAN 드라이버 설치 완료");
    } else {
        Serial.println("CAN 드라이버 설치 실패");
        return false;
    }

    if (twai_start() == ESP_OK) {
        Serial.println("CAN 드라이버 시작");
        return true;
    } else {
        Serial.println("CAN 드라이버 시작 실패");
        return false;
    }
}

bool can_receive_dht(float &humidity, float &temperature) {
    twai_message_t rx_msg;

    // 1000ms 대기 후 수신 없으면 타임아웃
    esp_err_t result = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));
    if (result != ESP_OK) {
        return false;
    }

    // RTR 프레임이면 무시
    if ((rx_msg.flags & TWAI_MSG_FLAG_RTR) != 0) {
        return false;
    }

    // ID 필터
    if (rx_msg.identifier != CAN_DHT_ID) {
        return false;
    }

    // 길이 확인 (습도 2바이트 + 온도 2바이트)
    if (rx_msg.data_length_code < 4) {
        Serial.println("RX frame too short");
        return false;
    }

    // little-endian 복원
    int16_t humidity_x10 = (int16_t)((rx_msg.data[1] << 8) | rx_msg.data[0]);
    int16_t temperature_x10 = (int16_t)((rx_msg.data[3] << 8) | rx_msg.data[2]);

    humidity = humidity_x10 / 10.0f;
    temperature = temperature_x10 / 10.0f;

    return true;
}

void setup()
{
    Serial.begin(115200);
    delay(300);

    bool can_ok = can_driver_init();
    if (!can_ok) {
        Serial.println("CAN init failed. Stop.");
        while (true) {
            delay(1000);
        }
    }

    Serial.println("RX ready");
}

void loop()
{
    float humidity = 0.0f;
    float temperature = 0.0f;

    bool received = can_receive_dht(humidity, temperature);
    if (received) {
        Serial.printf("RX: H=%.1f%% T=%.1fC\n", humidity, temperature);
    }
}
