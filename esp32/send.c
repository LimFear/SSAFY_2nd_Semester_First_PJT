#include <Arduino.h>
#include <math.h>
#include "driver/twai.h"
#include "DHT.h"

/* ===== CAN 통신 핀 설정 ===== */
#define CAN_RX_GPIO_PIN GPIO_NUM_32
#define CAN_TX_GPIO_PIN GPIO_NUM_33

/* ===== DHT11 설정 ===== */
#define DHTPIN 17
#define DHTTYPE DHT11

/* ===== CDS(조도, ADC) 설정 ===== */
#define CDS_GPIO_PIN 34

/* ===== CAN ID ===== */
#define CAN_DHT_ID 0x100
#define CAN_CDS_ID 0x110

static DHT g_dht(DHTPIN, DHTTYPE);

/* ===== CAN 기본 환경 설정 ===== */
static bool can_driver_init()
{
    twai_general_config_t generalConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_PIN, CAN_RX_GPIO_PIN, TWAI_MODE_NORMAL);

    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result = twai_driver_install(&generalConfig, &timingConfig, &filterConfig);
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

static void printTwaiStatus()
{
    twai_status_info_t statusInfo;
    esp_err_t result = twai_get_status_info(&statusInfo);
    if (result != ESP_OK) {
        Serial.printf("twai_get_status_info fail: %d\n", (int)result);
        return;
    }

    Serial.printf("TWAI state=%d tx_err=%u rx_err=%u msgs_to_tx=%u msgs_to_rx=%u bus_err=%u\n",
        (int)statusInfo.state,
        (unsigned)statusInfo.tx_error_counter,
        (unsigned)statusInfo.rx_error_counter,
        (unsigned)statusInfo.msgs_to_tx,
        (unsigned)statusInfo.msgs_to_rx,
        (unsigned)statusInfo.bus_error_count);
}


static bool can_send_dht(float humidity, float temperature)
{
    int16_t humidity_x10 = (int16_t)lroundf(humidity * 10.0f);
    int16_t temperature_x10 = (int16_t)lroundf(temperature * 10.0f);

    twai_message_t txMessage = {};
    txMessage.identifier = CAN_DHT_ID;
    txMessage.flags = TWAI_MSG_FLAG_NONE;
    txMessage.data_length_code = 4;

    txMessage.data[0] = (uint8_t)(humidity_x10 & 0xFF);
    txMessage.data[1] = (uint8_t)((humidity_x10 >> 8) & 0xFF);
    txMessage.data[2] = (uint8_t)(temperature_x10 & 0xFF);
    txMessage.data[3] = (uint8_t)((temperature_x10 >> 8) & 0xFF);

    esp_err_t result = twai_transmit(&txMessage, 0);
    if (result == ESP_OK) {
        return true;
    }

    Serial.printf("CAN transmit failed: %d\n", (int)result);
    printTwaiStatus();
    return false;
}

static bool can_send_cds_raw(uint16_t cdsAdc12)
{
    twai_message_t txMessage = {};
    txMessage.identifier = CAN_CDS_ID;
    txMessage.flags = TWAI_MSG_FLAG_NONE;
    txMessage.data_length_code = 2;

    txMessage.data[0] = (uint8_t)(cdsAdc12 & 0xFF);
    txMessage.data[1] = (uint8_t)((cdsAdc12 >> 8) & 0xFF);

    esp_err_t result = twai_transmit(&txMessage, 0);
    if (result == ESP_OK) {
        return true;
    }

    Serial.printf("CAN transmit(CDS) failed: %d\n", (int)result);
    printTwaiStatus();
    return false;
}



void setup()
{
    Serial.begin(115200);
    delay(300);

    bool canOk = can_driver_init();
    if (canOk == false) {
        Serial.println("CAN init failed. Stop.");
        while (true) { delay(1000); }
    }

    g_dht.begin();

    /* ESP32(일반 모델) 기준으로 GPIO17은 ADC 핀이 아님.
       실제 ADC 입력이 필요하면 32~39(ADC1) 또는 25~27(ADC2)로 옮겨야 함.
       사용자 요구사항에 따라 GPIO17로 유지함. */
    analogReadResolution(12);

    Serial.println("TX ready");
}

void loop()
{
    delay(2000);

    /* ===== DHT11 ===== */
    float humidity = g_dht.readHumidity();
    float temperature = g_dht.readTemperature();

    if (isnan(humidity) == false && isnan(temperature) == false) {
        bool sentDht = can_send_dht(humidity, temperature);
        if (sentDht) {
            Serial.printf("TX: H=%.1f%% T=%.1fC\n", humidity, temperature);
        }
    }
    else {
        Serial.println("Failed to read from DHT sensor!");
    }

    /* ===== CDS(조도, ADC raw) ===== */
    uint16_t cdsAdc12 = (uint16_t)analogRead((int)CDS_GPIO_PIN);

    bool sentCds = can_send_cds_raw(cdsAdc12);
    if (sentCds) {
        Serial.printf("TX: CDS(adc)=%u\n", (unsigned)cdsAdc12);
    }
}
