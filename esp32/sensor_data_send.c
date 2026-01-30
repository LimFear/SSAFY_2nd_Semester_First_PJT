#include <Arduino.h>
#include <math.h>
#include "driver/twai.h"
#include "DHT.h"

// CAN통신 핀 설정
#define CAN_RX_GPIO_PIN GPIO_NUM_32
#define CAN_TX_GPIO_PIN GPIO_NUM_33

// DHT11 설정
#define DHTPIN 16
#define DHTTYPE DHT11

// CAN ID
#define CAN_DHT_ID 0x100

DHT dht(DHTPIN, DHTTYPE);

// CAN 기본 환경 설정
bool can_driver_init() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_PIN, CAN_RX_GPIO_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);
    if (result != ESP_OK) {
        Serial.printf("CAN install fail: %d\n", (int)result);
        return false;
    }
    Serial.println("CAN install OK");

    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("CAN start fail: %d\n", (int)result);
        return false;
    }
    Serial.println("CAN start OK");
    return true;
}


bool can_send_dht(float humidity, float temperature) {
    // float 그대로 보내지 않고 int16으로 스케일링
    int16_t humidity_x10 = (int16_t)lroundf(humidity * 10.0f);       // 예: 55.3% -> 553
    int16_t temperature_x10 = (int16_t)lroundf(temperature * 10.0f); // 예: 24.1C -> 241

    twai_message_t tx_msg = {};
    tx_msg.identifier = CAN_DHT_ID;
    tx_msg.flags = TWAI_MSG_FLAG_NONE;   // 표준 프레임
    tx_msg.data_length_code = 4;

    // little-endian
    tx_msg.data[0] = (uint8_t)(humidity_x10 & 0xFF);
    tx_msg.data[1] = (uint8_t)((humidity_x10 >> 8) & 0xFF);
    tx_msg.data[2] = (uint8_t)(temperature_x10 & 0xFF);
    tx_msg.data[3] = (uint8_t)((temperature_x10 >> 8) & 0xFF);

    esp_err_t result = twai_transmit(&tx_msg, pdMS_TO_TICKS(200));
    if (result == ESP_OK) {
        return true;
    }

    // 상대 노드/종단저항/배선 문제로 ACK가 없으면 여기서 실패할 수 있음
    Serial.printf("CAN transmit failed: %d\n", (int)result);
    return false;
}

void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);

    // CAN 시작
    bool can_ok = can_driver_init();
    if (!can_ok)
    {
        Serial.println("CAN init failed. Stop.");
        while (true)
        {
            delay(1000);
        }
    }

    //DHT11 모듈 시작
    dht.begin();
    Serial.println("TX ready");
}

void loop() {
    // put your main code here, to run repeatedly:

    delay(2000);
    //DHT11 센서로부터 데이터 읽음
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();

    // 읽기 실패할 때
    if (isnan(humidity) || isnan(temperature))
    {
        Serial.println(F("Failed to read from DHT sensor!"));
        return;
    }

    bool sent = can_send_dht(humidity, temperature);

    if (sent) {
        Serial.printf("TX: H=%.1f%% T=%.1fC\n", humidity, temperature);
    }

}
