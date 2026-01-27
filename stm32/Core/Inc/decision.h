#pragma once
#include <stdint.h>
#include <string.h>

/**
 * [SSAFY 2학기 공통PJT - 통합 프로토콜 헤더 (Gateway용)]
 * 
 * 이 파일은 게이트웨이(STM32)가 수신한 CAN 메시지를 해석하고, 무결성을 검증하는 '판단 기준'이 담겨 있어.
 * 운영 정책 v2.2의 E2E-lite 규격을 이 소스 함수들이 담당해!
 */

/* ===== CAN ID v2.2 규격 (정책 문서 준수) ===== */
#define CAN_ID_GW_ESTOP                (0x080U) // 긴급 정지 / 안전 선언
#define CAN_ID_CMD_LIGHT               (0x110U) // 조명 제어 명령
#define CAN_ID_CMD_WIPER               (0x120U) // 와이퍼 제어 명령
#define CAN_ID_ACT_FEEDBACK            (0x130U) // 액츄에이터 실행 결과 수신
#define CAN_ID_GW_STATE                (0x140U) // 게이트웨이 상태 브로드캐스트
#define CAN_ID_SNS_ENV                 (0x210U) // 온습도 수신
#define CAN_ID_SNS_LUX                 (0x220U) // 조도 수신
#define CAN_ID_SNS_ALIVE               (0x2F0U) // 센서 생존 확인

/* ===== 제어 대상 코드 ===== */
#define TARGET_HEADLIGHT               (0x01U)
#define TARGET_TAILLIGHT               (0x02U)
#define TARGET_WIPER                   (0x03U)

// CAN 로우 데이터 프레임 구조체
typedef struct
{
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} CanRawFrame_t;

// 수집된 센서 데이터 모음
typedef struct
{
    float humidity;
    float temperature;
    uint16_t lux;
    uint32_t tick_ms; // 수신된 시간 (타임아웃 체크용)
} SensorSample_t;

// 생성할 제어 명령 구조체
typedef struct
{
    uint8_t target;
    uint8_t action; // 켜기/끄기 혹은 모드(LOW/HIGH)
    uint8_t level;
    uint8_t token;  // 명령 추적을 위한 고유 토큰
} ControlCommand_t;

/**
 * [v2.2 E2E-lite] CRC8 계산 함수
 * ESP32가 보낸 데이터와 내(Gateway)가 계산한 값이 일치해야만 "진짜 데이터"로 인정해!
 * 하드웨어가 계산해주는 CRC와 별개로, 소프트웨어 레벨에서 한 번 더 검증하는 기능 안전(ISO 26262) 방식이야.
 */
static inline uint8_t protocol_calculate_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++)
    {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x80) crc = (crc << 1) ^ 0x07;
            else crc <<= 1;
        }
    }
    return crc;
}

/**
 * 온습도 데이터 파서 (v2.2 반영)
 * 단순히 값을 읽는 게 아니라, CRC8과 Sender ID(누가 보냈는지)를 먼저 검사해.
 */
static inline int protocol_parse_env_v22(const uint8_t *data, uint8_t dlc, float *h, float *t)
{
    if (dlc < 8) return 0; // 데이터 길이가 너무 짧으면 오류
    
    // 1. 무결성 검사: B7에 실려온 CRC8이 내가 계산한 값과 같은가?
    if (data[7] != protocol_calculate_crc8(data, 7)) return -1; // 위변조 위험 데이터! 파기!
    
    // 2. 출처 검사: B0에 적힌 Sender ID가 '센서 노드(1)'가 맞는가?
    if (data[0] != 1) return -2; // 모르는 녀석이 보낸 데이터! 파기!
    
    // 3. 값 추출: 온도는 정밀도를 위해 x100 되어 있으므로 다시 100으로 나눠줘.
    int16_t t_raw = (int16_t)((data[2] << 8) | data[3]);
    *t = (float)t_raw / 100.0f;
    *h = (float)data[4];
    
    return 1; // 성공적으로 해석 완료!
}

/**
 * 조도 데이터 파서 (v2.2 반영)
 * 환경 데이터와 마찬가지로 무결성 검증을 가장 먼저 수행해.
 */
static inline int protocol_parse_lux_v22(const uint8_t *data, uint8_t dlc, uint16_t *lux)
{
    if (dlc < 8) return 0;
    
    if (data[7] != protocol_calculate_crc8(data, 7)) return -1;
    if (data[0] != 1) return -2;
    
    // 조도 값은 2바이트(B2~3)에 Big-Endian(상위바이트 먼저) 방식으로 담겨 있어.
    *lux = (uint16_t)((data[2] << 8) | data[3]);
    return 1;
}
