#include <Arduino.h>
#include "driver/twai.h"

/* 
 * [SSAFY 2학기 공통PJT - 통합 프로젝트 (Teammate Actuator Node)]
 * 
 * 이 코드는 "게이트웨이의 명령(LED, 와이퍼) 수신 및 안전 감시"를 담당해.
 * 운영 정책 v2.2에 따라 '게이트웨이가 고장 났을 때' 스스로 안전해지는 로직이 추가되었어!
 */

/* ===== 핀 및 상수 설정 ===== */
#define CAN_TX_PIN      33
#define CAN_RX_PIN      32
#define SERVO_PIN       17  // 와이퍼 제어용 서보 모터
#define LED_R_PIN       4   // 전조등 대용 RGB LED (R)
#define LED_G_PIN       5   
#define LED_B_PIN       18  

/* ===== CAN ID 설정 (운영 정책 v2.2 준수) ===== */
#define ID_CMD_LIGHT    0x110 // 게이트웨이가 보내는 조명 명령
#define ID_CMD_WIPER    0x120 // 게이트웨이가 보내는 와이퍼 명령
#define ID_ACT_FEEDBACK 0x130 // [필수] 우리가 수행한 결과를 다시 보고하는 채널
#define ID_GW_STATE     0x140 // 게이트웨이가 정기적으로 보내는 본인의 건강 상태 신호

// 타이머 변수: 게이트웨이가 살아있는지 감시하는 용도야
unsigned long last_cmd_tick = 0;
#define T_DEAD_MS 3000 // 3초 동안 게이트웨이 신호가 없으면 "비상"으로 판단해!

void setup() {
  Serial.begin(115200);
  
  // LED와 서보 등 장치 핀 초기화
  pinMode(LED_R_PIN, OUTPUT);
  pinMode(LED_G_PIN, OUTPUT);
  pinMode(LED_B_PIN, OUTPUT);

  // CAN(TWAI) 드라이버 설정 (500kbps)
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    twai_start();
    Serial.println("Actuator Node v2.2 (Safety First) Started!");
    last_cmd_tick = millis(); // 지금부터 감시 시작!
  }
}

void loop() {
  twai_message_t rx_msg;
  
  // --- 1. 명령 수신 루프 (상시 수신) ---
  // 10ms마다 메시지가 왔는지 체크해.
  if (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK) {
    uint8_t token = 0;      // 명령의 고유 번호 (우리 답장에 꼭 실어 보내야 해)
    bool needs_feedback = false;
    uint8_t target = 0;     // 제어 대상 (1=HEAD, 3=WIPER 등)
    uint8_t current_state = 0;

    // [v2.2 안전 로직] 명령어나 게이트웨이 상태 신호를 받으면 "게이트웨이가 살아있구나!"라고 판단해서 감시 타이머를 리셋해.
    if (rx_msg.identifier == ID_CMD_LIGHT || rx_msg.identifier == ID_CMD_WIPER || rx_msg.identifier == ID_GW_STATE) {
      last_cmd_tick = millis();
    }

    // A. 조명 제어 명령 (0x110)
    if (rx_msg.identifier == ID_CMD_LIGHT) {
      target = rx_msg.data[0];    // B0: 대상 (1=HEADLIGHT)
      uint8_t on_off = rx_msg.data[1]; // B1: 0=OFF, 1=ON
      token = rx_msg.data[3];     // B3: 명령 토큰 (답장할 때 똑같이 써야 함)
      
      if (target == 1) { 
        digitalWrite(LED_R_PIN, on_off); // 전조등 켜기/끄기
        current_state = on_off;
      }
      needs_feedback = true;
    } 
    // B. 와이퍼 제어 명령 (0x120)
    else if (rx_msg.identifier == ID_CMD_WIPER) {
      target = 3;                 // B0: 대상 (3=WIPER)
      uint8_t mode = rx_msg.data[0];   // B0: 0=OFF, 1=LOW, 2=HIGH
      token = rx_msg.data[3];     // B3: 토큰
      
      // [참고] 여기에 서보 모터를 모드에 맞춰 움직이는 코드를 넣어줘!
      current_state = mode;
      needs_feedback = true;
    }

    // --- 2. 피드백 전송 (ACT_FEEDBACK) ---
    // 명령을 성공적으로 처리했다면, 게이트웨이에게 "나 잘했어!"라고 알려줘야 해.
    // 게이트웨이는 이 답장이 없으면 비상 버튼(ESTOP)을 눌러버릴 수도 있어.
    if (needs_feedback) {
      twai_message_t tx_fb = {.identifier = ID_ACT_FEEDBACK, .data_length_code = 4};
      tx_fb.data[0] = target;        // 내가 제어한 장치 (1=HEAD, 3=WIPER)
      tx_fb.data[1] = 0;             // 결과: 0=OK (성공)
      tx_fb.data[2] = current_state; // 현재 장치 상태 정보
      tx_fb.data[3] = token;         // [매우 중요] 받은 토큰을 그대로 다시 보내기 (Token Echo)
      
      twai_transmit(&tx_fb, pdMS_TO_TICKS(10));
    }
  }

  // --- 3. [신규] 독립 FAILSAFE 로직 (v2.2 핵심) ---
  // 게이트웨이가 죽어서(혹은 통신이 끊겨서) 명령이 3초 동안 안 오면 어떻게 될까?
  // 운전자의 안전을 위해 모든 장치를 안전한 기본 상태(OFF)로 돌려야 해!
  if (millis() - last_cmd_tick > T_DEAD_MS) {
    // 3초 동안 침묵 -> 비상 모드 진동!!
    digitalWrite(LED_R_PIN, LOW); // 조명 모두 끄기
    digitalWrite(LED_G_PIN, LOW);
    digitalWrite(LED_B_PIN, LOW);
    // 와이퍼도 정지 위치로 이동시키는 코드를 여기에 추가할 수 있어.
    
    Serial.println("!!! DANGER !!! Gateway is silent. Entering FAILSAFE mode for safety.");
    
    // 알림이 너무 도배되지 않게 타이머를 살짝 밀어줘
    last_cmd_tick = millis(); 
  }
}
