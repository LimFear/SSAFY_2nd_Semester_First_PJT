#include <Arduino.h>
#include "driver/twai.h"
#include <DHT.h>

/* 
 * [SSAFY 2학기 공통PJT - 통합 프로젝트 (Teammate Sensor Node)]
 * 
 * 이 코드는 "센서 데이터 수집 및 송신"을 담당해.
 * 기존 DHT11 코드에서 운영 정책 v2.2(무결성 강화)를 위해 많은 부분이 발전했어!
 */

/* ===== 핀 및 상수 설정 ===== */
#define CAN_TX_PIN      33
#define CAN_RX_PIN      32
#define DHT_PIN         14  // [중요] 기존 3번에서 14번으로 옮겼어 (시리얼 통신 간섭 방지)
#define CDS_PIN         34  // 조도 센서(CDS) ADC 입력 핀

/* ===== CAN ID 설정 (운영 정책 v2.2 준수) ===== */
#define ID_SNS_ENV      0x210 // 온습도 데이터 전송용
#define ID_SNS_LUX      0x220 // 조도 데이터 전송용
#define ID_SNS_ALIVE    0x2F0 // 노드가 살아있음을 알리는 심박(Heartbeat) 신호

/**
 * [신규] E2E-lite CRC8 체크섬 계산 함수
 * 자동차 전장 시스템에서는 전송 중에 데이터가 깨지는 것을 막기 위해 체크섬이 필수야.
 * 이 함수는 0x07 다항식을 사용하여 7바이트 데이터를 연산한 뒤 1바이트 결과값을 만들어내.
 * 게이트웨이는 이 값을 똑같이 계산해서 네가 보낸 데이터가 진짜인지 검증할 거야.
 */
uint8_t calculate_crc8(uint8_t *data, uint8_t len) {
  uint8_t crc = 0x00;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 0x80) crc = (crc << 1) ^ 0x07;
      else crc <<= 1;
    }
  }
  return crc;
}

DHT dht(DHT_PIN, DHT11);

void setup() {
  Serial.begin(115200);
  dht.begin();
  pinMode(CDS_PIN, INPUT);

  // CAN(TWAI) 드라이버 설정 (기본 500kbps 사용)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
    Serial.println("Sensor Node v2.2 (ISO Standard) Started!");
  }
}

// 전역 변수들: 데이터의 순서와 송신 주기를 관리해
uint8_t rolling_cnt = 0; // 메시지 송신 때마다 1씩 증가하는 카운터 (중복/누락 체크용)
uint8_t alive_cnt = 0;   // 1초마다 보낼 생존 신호용 카운터
unsigned long last_send_tick = 0; // delay() 대신 사용할 시간 체크 변수

void loop() {
  // [논블로킹 방식] millis()를 시계처럼 사용해서 500ms(0.5초)마다 센서 값을 쏠 거야.
  // delay()를 쓰면 데이터를 보내는 동안 명령 수신을 못 하니까 꼭 이 방식을 써야 해!
  if (millis() - last_send_tick >= 500) {
    last_send_tick = millis();

    // --- 1. 온습도 데이터 전송 (SNS_ENV) ---
    float h = dht.readHumidity();
    float t = dht.readTemperature();
    
    if (!isnan(h) && !isnan(t)) {
      twai_message_t msg_env = {.identifier = ID_SNS_ENV, .data_length_code = 8};
      
      // [v2.2 규격 패킹]
      msg_env.data[0] = 1;             // B0: Sender ID (1 = 나는 센서 노드다)
      msg_env.data[1] = rolling_cnt++; // B1: 순서 카운터 (메시지 쏠 때마다 +1)
      
      // B2~3: 온도를 소수점까지 살리기 위해 100을 곱해서 2바이트로 보내
      int16_t t_x100 = (int16_t)(t * 100);
      msg_env.data[2] = (t_x100 >> 8); // 상위 8비트
      msg_env.data[3] = (t_x100 & 0xFF); // 하위 8비트
      
      msg_env.data[4] = (uint8_t)h;    // B4: 습도 정수값
      msg_env.data[5] = 0;             // 예비 필드
      msg_env.data[6] = 0;             // 예비 필드
      
      // B7: 무결성 검증을 위한 CRC8 (앞선 7바이트를 요리해서 1바이트를 만듦)
      msg_env.data[7] = calculate_crc8(msg_env.data, 7); 
      
      twai_transmit(&msg_env, 0); // 즉시 송신
    }

    // --- 2. 조도(CDS) 데이터 전송 (SNS_LUX) ---
    int lux_raw = analogRead(CDS_PIN);
    twai_message_t msg_lux = {.identifier = ID_SNS_LUX, .data_length_code = 8};
    
    msg_lux.data[0] = 1;             // B0: Sender ID
    msg_lux.data[1] = rolling_cnt++; // B1: 순서 카운너
    
    // B2~3: 조도 Raw ADC 값을 2바이트에 담아 (게이트웨이가 변환할 거야)
    msg_lux.data[2] = (lux_raw >> 8);
    msg_lux.data[3] = (lux_raw & 0xFF);
    
    msg_lux.data[4] = 0;             // 예비 필드
    msg_lux.data[7] = calculate_crc8(msg_lux.data, 7); // 무결성 체크
    
    twai_transmit(&msg_lux, 0);

    // --- 3. 생존 신호 전송 (SNS_ALIVE) ---
    // 500ms 루프가 2번 돌 때마다 (즉, 1초마다) "나 안 죽고 살아있어"라고 알려줘.
    if (alive_cnt % 2 == 0) {
      twai_message_t msg_alive = {.identifier = ID_SNS_ALIVE, .data_length_code = 2};
      msg_alive.data[0] = 1; // 내 ECU ID
      msg_alive.data[1] = alive_cnt / 2; // 생존 횟수 카운트
      twai_transmit(&msg_alive, 0);
    }
    alive_cnt++;
    
    Serial.print("Data Sent to Gateway (T="); Serial.print(t); 
    Serial.print(" H="); Serial.print(h); 
    Serial.print(" LUX="); Serial.println(lux_raw);
  }
}
