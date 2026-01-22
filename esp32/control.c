#include <Arduino.h>            // Arduino 기본 API를 사용하기 위한 헤더임
#include "driver/twai.h"        // ESP32 TWAI(CAN) 드라이버를 사용하기 위한 헤더임

/* ===== CAN 핀(사용중인 핀으로 수정) ===== */
#define CAN_RX_GPIO_PIN GPIO_NUM_32   // CAN 수신 핀을 지정하는 매크로임
#define CAN_TX_GPIO_PIN GPIO_NUM_33   // CAN 송신 핀을 지정하는 매크로임

/* ===== 서보 핀(원하는 핀으로 수정) ===== */
#define SERVO_GPIO_PIN  (GPIO_NUM_17) // 서보 PWM 출력 핀을 지정하는 매크로임

/* ===== CAN 프로토콜 ===== */
#define CAN_ID_CMD_SERVO        0x200 // STM32->ESP32(서보노드) 명령 프레임의 CAN ID임
#define SERVO_CMD_SET_ANGLE     0x01  // data[0]에 들어가는 “서보 제어 명령” 코드임(기존 재사용)

/*
  ===== 이번 설계에서의 data[1] 의미 =====
  - data[1]을 "각도"가 아니라 "속도 레벨(speed_level)"로 사용함
  - speed_level:
      0 = STOP   : 스윙 중단 + 0도로 복귀 후 정지
      1 = SLOW   : 습도 >= 30 구간 (느리게)
      2 = NORMAL : 습도 >= 35 구간 (현재 속도)
      3 = FAST   : 습도 >= 40 구간 (가장 빠르되 약간 여유)
*/

/* ===== 서보 최대 각도 요구사항 ===== */
static const uint8_t kMaxAngleLimit = 125;   // 스윙 상한 각도를 125도로 고정하기 위한 상수임

/* ===== LEDC(서보 PWM) 설정 ===== */
static const int kPwmFreqHz = 50;            // 서보는 보통 50Hz(20ms) PWM을 사용하기 때문에 50으로 둠
static const int kPwmResolutionBits = 16;    // duty 분해능을 16bit로 설정하여 미세 제어가 가능하게 함

/* ===== RTOS ===== */
static QueueHandle_t g_speedLevelQueue;      // CAN 수신(speed_level)을 서보 태스크로 전달하는 큐 핸들임

/* ===== 속도 레벨 상수 ===== */
static const uint8_t kSpeedLevelStop   = 0;  // STOP 레벨(스윙 중단 및 0도 복귀)임
static const uint8_t kSpeedLevelSlow   = 1;  // 느린 스윙 레벨(습도 30 이상)임
static const uint8_t kSpeedLevelNormal = 2;  // 기본 스윙 레벨(습도 35 이상, 현재 속도)임
static const uint8_t kSpeedLevelFast   = 3;  // 빠른 스윙 레벨(습도 40 이상)임

/* ===== 서보 속도 튜닝 파라미터(5V 기준 시작값) =====
   - stepSweepDeg: 20ms마다 몇 도씩 이동할지 결정함(클수록 더 빠르게 왕복함)
   - dwellDelay: 끝점(0도/125도)에서 멈추는 시간임(클수록 더 “느린 느낌”이 남)
*/
static const uint8_t kStepSweepSlowDeg   = 3;                 // SLOW에서 20ms마다 3도 이동하도록 설정함
static const uint8_t kStepSweepNormalDeg = 5;                 // NORMAL에서 20ms마다 5도 이동(정환님 현재 속도)하도록 설정함
static const uint8_t kStepSweepFastDeg   = 8;                 // FAST에서 20ms마다 8도 이동(최고속보다 약간 여유)하도록 설정함

static const TickType_t kDwellSlow  = pdMS_TO_TICKS(120);      // SLOW 끝점 정지 시간을 120ms로 둠
static const TickType_t kDwellNormal= pdMS_TO_TICKS(80);       // NORMAL 끝점 정지 시간을 80ms로 둠
static const TickType_t kDwellFast  = pdMS_TO_TICKS(40);       // FAST 끝점 정지 시간을 40ms로 둠

static const uint8_t kStepStopDeg   = 10;                     // STOP 복귀 시 20ms마다 10도씩 내려 빠르게 0도로 초기화되게 함
static const TickType_t kFrameDelay = pdMS_TO_TICKS(20);       // 50Hz 프레임에 맞춰 20ms 주기로 동작시키기 위한 지연임

/* ===== 통신 끊김 대비 타임아웃(선택 사항이지만 추천) ===== */
static const TickType_t kCommandTimeout = pdMS_TO_TICKS(5000); // 5초 동안 speed_level이 안 오면 STOP으로 복귀시키는 타임아웃임

/* 50Hz(20000us)에서 pulse_us를 duty로 변환 */
static uint32_t servoDutyFromPulseUs(uint32_t pulse_us)
{
    const uint32_t period_us = 20000U;                        // 50Hz의 주기는 20000us이므로 이를 기준으로 duty를 계산함

    if (pulse_us > period_us) {                               // 펄스 폭이 주기를 넘지 않도록 상한을 강제함
        pulse_us = period_us;                                 // 상한을 넘는 경우 period_us로 클램핑함
    }

    const uint32_t maxDuty = (1U << kPwmResolutionBits) - 1U;  // 16bit 분해능에서의 최대 duty 값을 계산함
    uint32_t duty = (pulse_us * maxDuty) / period_us;         // pulse_us를 duty로 선형 변환함
    return duty;                                              // 계산된 duty를 반환함
}

static uint32_t pulseUsFromAngle(uint8_t angle_deg)
{
    if (angle_deg > 180U) {                                   // 서보 입력 각도를 0~180 범위로 제한함
        angle_deg = 180U;                                     // 180을 넘으면 180으로 클램핑함
    }

    /* 일반 서보 범위(서보마다 다를 수 있음) */
    const uint32_t min_us = 500U;                             // 보편적인 최소 펄스 폭(0도 근처)임
    const uint32_t max_us = 2500U;                            // 보편적인 최대 펄스 폭(180도 근처)임

    uint32_t pulse = min_us + (uint32_t)((max_us - min_us) * angle_deg) / 180U; // 각도를 펄스폭으로 선형 매핑함
    return pulse;                                             // 계산된 펄스 폭(us)을 반환함
}

static void servoInit()
{
    bool ok = ledcAttach((int)SERVO_GPIO_PIN, kPwmFreqHz, kPwmResolutionBits); // 핀 기반 LEDC를 설정하여 PWM을 출력 가능하게 함
    if (!ok) {                                                                // attach 실패 시 진단 로그를 남김
        Serial.println("LEDC attach failed");                                 // LEDC 초기화 실패 메시지를 출력함
    }
}

static void servoWriteAngle(uint8_t angle_deg)
{
    uint32_t pulse = pulseUsFromAngle(angle_deg);              // 입력 각도를 PWM 펄스 폭(us)으로 변환함
    uint32_t duty  = servoDutyFromPulseUs(pulse);              // 펄스 폭(us)을 duty 값으로 변환함

    /* 신규 API: 채널이 아니라 핀에 씀 */
    ledcWrite((int)SERVO_GPIO_PIN, duty);                      // 해당 핀에 duty를 써서 서보 각도를 갱신함
}

/* ===== speed_level -> step/dwell 변환 함수 ===== */
static uint8_t stepDegFromSpeedLevel(uint8_t speedLevel)
{
    if (speedLevel == kSpeedLevelSlow) {                       // 느린 레벨이면 작은 step을 사용함
        return kStepSweepSlowDeg;                              // SLOW step 값을 반환함
    }

    if (speedLevel == kSpeedLevelNormal) {                     // 기본 레벨이면 현재 속도를 유지함
        return kStepSweepNormalDeg;                            // NORMAL step 값을 반환함
    }

    if (speedLevel == kSpeedLevelFast) {                       // 빠른 레벨이면 큰 step을 사용함
        return kStepSweepFastDeg;                              // FAST step 값을 반환함
    }

    return kStepSweepNormalDeg;                                // 예외값은 기본 속도로 처리함
}

static TickType_t dwellDelayFromSpeedLevel(uint8_t speedLevel)
{
    if (speedLevel == kSpeedLevelSlow) {                       // 느린 레벨이면 끝점에서 더 오래 멈춤
        return kDwellSlow;                                     // SLOW 끝점 지연을 반환함
    }

    if (speedLevel == kSpeedLevelNormal) {                     // 기본 레벨이면 현재 끝점 지연을 유지함
        return kDwellNormal;                                   // NORMAL 끝점 지연을 반환함
    }

    if (speedLevel == kSpeedLevelFast) {                       // 빠른 레벨이면 끝점 멈춤을 줄여 체감 속도를 올림
        return kDwellFast;                                     // FAST 끝점 지연을 반환함
    }

    return kDwellNormal;                                       // 예외값은 기본 끝점 지연으로 처리함
}

/* ===== CAN 초기화 ===== */
static bool canInit()
{
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_PIN, CAN_RX_GPIO_PIN, TWAI_MODE_NORMAL); // TWAI 일반 설정(핀/모드)을 구성함
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();                      // CAN 비트레이트를 500kbps로 설정함
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();                    // 필터를 전부 수용하도록 설정함

    esp_err_t result = twai_driver_install(&g_config, &t_config, &f_config);            // TWAI 드라이버를 설치하여 CAN을 사용 가능하게 함
    if (result != ESP_OK) {                                                             // 설치 실패 시 에러 로그를 남김
        Serial.printf("CAN install fail: %d\n", (int)result);                           // 설치 실패 원인을 코드로 출력함
        return false;                                                                   // 실패를 반환해 setup에서 정지 처리하게 함
    }

    result = twai_start();                                                              // CAN 컨트롤러를 실제로 시작함
    if (result != ESP_OK) {                                                             // 시작 실패 시 에러 로그를 남김
        Serial.printf("CAN start fail: %d\n", (int)result);                             // 시작 실패 원인을 코드로 출력함
        return false;                                                                   // 실패를 반환해 setup에서 정지 처리하게 함
    }

    return true;                                                                        // CAN 초기화 성공을 반환함
}

/* ===== CAN RX Task ===== */
static void canRxTask(void *param)
{
    (void)param;                                                                        // 사용하지 않는 인자를 명시적으로 무시함

    Serial.println("canRxTask start");                                                  // CAN 수신 태스크가 시작되었음을 출력함

    for (;;)
    {
        twai_message_t rx_msg;                                                          // 수신 CAN 프레임을 담을 구조체임
        esp_err_t result = twai_receive(&rx_msg, pdMS_TO_TICKS(1000));                   // 최대 1초 동안 수신을 대기함
        if (result != ESP_OK) {                                                         // 타임아웃/에러면 다음 루프로 넘어감
            continue;                                                                   // 수신 실패 시 처리 부담을 줄이기 위해 skip함
        }

        if (rx_msg.flags & TWAI_MSG_FLAG_EXTD) {                                        // 확장 ID 프레임이면 무시함(표준 ID만 사용)
            continue;                                                                   // 확장 ID를 사용하지 않으므로 제외함
        }

        if (rx_msg.identifier != CAN_ID_CMD_SERVO) {                                    // 서보 명령 ID(0x200)가 아니면 무시함
            continue;                                                                   // 다른 장치 프레임과 분리하기 위해 skip함
        }

        if (rx_msg.data_length_code < 2) {                                              // 최소 2바이트(cmd + level)가 없으면 무시함
            continue;                                                                   // 프로토콜 길이가 부족하므로 제외함
        }

        uint8_t command = rx_msg.data[0];                                               // data[0]은 명령 코드이므로 이를 읽음
        uint8_t speedLevel = rx_msg.data[1];                                            // data[1]은 speed_level이므로 이를 읽음

        if (command != SERVO_CMD_SET_ANGLE) {                                           // 우리가 정의한 명령 코드가 아니면 무시함
            continue;                                                                   // 다른 cmd는 현재 구현에서 처리하지 않음
        }

        if (speedLevel > kSpeedLevelFast) {                                             // speed_level 범위(0~3)를 넘으면 보정함
            speedLevel = kSpeedLevelFast;                                               // 범위를 넘는 값은 FAST로 클램핑함
        }

        Serial.printf("[RX] id=0x%X cmd=%u speed_level=%u\n",
                      (unsigned)rx_msg.identifier,                                      // 수신된 CAN ID를 출력함
                      (unsigned)command,                                                // 수신된 command를 출력함
                      (unsigned)speedLevel);                                            // 수신된 speed_level을 출력함

        /* 최신값만 유지: 큐가 꽉 차면 1개 버리고 넣기 */
        if (uxQueueSpacesAvailable(g_speedLevelQueue) == 0)                             // 큐가 꽉 찬 경우 최신값 유지 정책을 적용함
        {
            uint8_t discarded;                                                          // 버릴 이전 값을 담을 임시 변수임
            (void)xQueueReceive(g_speedLevelQueue, &discarded, 0);                      // 큐에서 1개를 꺼내 비워줌
        }

        (void)xQueueSend(g_speedLevelQueue, &speedLevel, 0);                            // 최신 speed_level을 큐에 넣어 서보 태스크가 반영하게 함
    }
}

/* ===== Servo Task ===== */
enum ServoMode
{
    SERVO_MODE_STOP  = 0,                                                               // STOP 모드는 0도로 복귀 후 정지하는 상태임
    SERVO_MODE_SWEEP = 1,                                                               // SWEEP 모드는 0↔125를 반복 왕복하는 상태임
};

static void servoTask(void *param)
{
    (void)param;                                                                        // 사용하지 않는 인자를 명시적으로 무시함

    Serial.println("servoTask start");                                                  // 서보 제어 태스크가 시작되었음을 출력함

    ServoMode mode = SERVO_MODE_STOP;                                                   // 시작은 STOP 상태로 두어 안전하게 시작함

    uint8_t currentAngleDeg = 0;                                                        // 현재 서보 각도를 저장하는 변수임
    uint8_t maxAngleDeg = kMaxAngleLimit;                                                // 스윙 상한을 125도로 고정하는 변수임
    bool goingUp = true;                                                                // 현재 상승(0->125) 방향인지 나타내는 변수임

    uint8_t currentSpeedLevel = kSpeedLevelStop;                                        // 현재 적용된 speed_level을 저장하는 변수임
    uint8_t stepSweepDeg = stepDegFromSpeedLevel(currentSpeedLevel);                    // speed_level에 대응하는 step 값을 저장함
    TickType_t dwellDelay = dwellDelayFromSpeedLevel(currentSpeedLevel);                // speed_level에 대응하는 끝점 지연을 저장함

    TickType_t lastCommandTick = xTaskGetTickCount();                                   // 마지막 명령을 받은 시각을 기록해 타임아웃에 사용함

    servoWriteAngle(currentAngleDeg);                                                   // 초기 각도(0도)를 PWM에 반영함

    for (;;)
    {
        /* 새 명령 확인(있으면 즉시 반영) */
        uint8_t newSpeedLevel;                                                          // 큐에서 꺼낼 새 speed_level 변수임
        if (xQueueReceive(g_speedLevelQueue, &newSpeedLevel, 0) == pdTRUE)              // 큐에 값이 있으면 즉시 가져옴(논블로킹)
        {
            lastCommandTick = xTaskGetTickCount();                                      // 명령을 받았으므로 타임아웃 타이머를 갱신함

            if (newSpeedLevel == kSpeedLevelStop)                                       // STOP이면 스윙을 중단하고 0도로 복귀해야 함
            {
                mode = SERVO_MODE_STOP;                                                 // 동작 모드를 STOP으로 전환함
                currentSpeedLevel = kSpeedLevelStop;                                    // 현재 레벨도 STOP으로 갱신함
                stepSweepDeg = stepDegFromSpeedLevel(currentSpeedLevel);                // STOP에서도 값은 유지하지만 SWEEP에서만 실제 사용됨
                dwellDelay = dwellDelayFromSpeedLevel(currentSpeedLevel);               // STOP에서도 값은 유지하지만 SWEEP에서만 실제 사용됨
                Serial.println("[APPLY] STOP -> return to 0");                          // STOP 적용 로그를 출력함
            }
            else
            {
                mode = SERVO_MODE_SWEEP;                                                // STOP이 아니면 스윙 모드로 전환함
                currentSpeedLevel = newSpeedLevel;                                      // 현재 speed_level을 갱신함
                stepSweepDeg = stepDegFromSpeedLevel(currentSpeedLevel);                // speed_level에 맞게 스윙 step을 갱신함
                dwellDelay = dwellDelayFromSpeedLevel(currentSpeedLevel);               // speed_level에 맞게 끝점 지연을 갱신함
                goingUp = true;                                                         // 스윙을 시작할 때는 상승 방향으로 두어 일관성 있게 함

                Serial.printf("[APPLY] SWEEP 0 <-> %u, level=%u, step=%u, dwell=%lu\n",
                              (unsigned)maxAngleDeg,                                    // 스윙 상한(125)을 출력함
                              (unsigned)currentSpeedLevel,                              // 적용된 speed_level을 출력함
                              (unsigned)stepSweepDeg,                                   // 적용된 step 값을 출력함
                              (unsigned long)(dwellDelay * portTICK_PERIOD_MS));        // 적용된 끝점 지연(ms)을 출력함
            }
        }

        /* 명령이 끊기면 안전하게 STOP으로 복귀시키는 타임아웃 로직임 */
        // TickType_t nowTick = xTaskGetTickCount();                                       // 현재 tick을 읽어 타임아웃 판단에 사용함
        // if ((nowTick - lastCommandTick) > kCommandTimeout)                              // 마지막 명령 이후 5초가 지나면 failsafe를 수행함
        // {
        //     if (mode != SERVO_MODE_STOP)                                                // 이미 STOP이면 중복 수행을 막기 위해 검사함
        //     {
        //         mode = SERVO_MODE_STOP;                                                 // 안전하게 STOP 모드로 전환함
        //         currentSpeedLevel = kSpeedLevelStop;                                    // speed_level도 STOP으로 강제함
        //         Serial.println("[FAILSAFE] cmd timeout -> STOP");                       // 타임아웃에 의한 STOP을 로그로 남김
        //     }
        // }

        if (mode == SERVO_MODE_STOP)
        {
            /* 반드시 0도로 복귀해서 “완전 종료” */
            if (currentAngleDeg == 0U)                                                  // 이미 0도면 유지하며 CPU 점유를 낮춤
            {
                vTaskDelay(kFrameDelay);                                                // 프레임 주기만큼 쉬어 CPU 점유를 줄임
                continue;                                                               // 다음 루프로 넘어가며 STOP 상태를 유지함
            }

            if (currentAngleDeg > kStepStopDeg)                                         // 0도로 내려갈 때 stepStop만큼 감소시킴
            {
                currentAngleDeg = (uint8_t)(currentAngleDeg - kStepStopDeg);            // 복귀 속도를 올리기 위해 큰 step으로 감소시킴
            }
            else
            {
                currentAngleDeg = 0U;                                                   // 남은 각도가 작으면 0도로 스냅하여 완전 종료시킴
            }

            servoWriteAngle(currentAngleDeg);                                           // 계산된 각도를 PWM에 반영하여 실제 서보를 움직임

            if (currentAngleDeg == 0U)                                                  // 0도에 도달하면 “완전 종료” 로그를 찍음
            {
                Serial.println("[DONE] servo at 0");                                    // 종료 느낌을 확실히 주기 위한 완료 로그임
            }

            vTaskDelay(kFrameDelay);                                                    // 프레임 주기만큼 쉬어 CPU 점유를 줄임
            continue;                                                                   // STOP 동작을 마치고 다음 루프로 넘어감
        }

        /* SERVO_MODE_SWEEP: 0 <-> 125 왕복 스윙 */
        if (goingUp)
        {
            uint16_t next = (uint16_t)currentAngleDeg + (uint16_t)stepSweepDeg;         // 상승 방향일 때 step만큼 각도를 증가시킴
            if (next >= maxAngleDeg)                                                    // 상한에 도달하면 방향을 전환함
            {
                currentAngleDeg = maxAngleDeg;                                          // 상한(125)에서 정확히 멈추게 함
                goingUp = false;                                                        // 다음 루프부터 하강 방향으로 바꿈
                servoWriteAngle(currentAngleDeg);                                       // 상한 각도를 PWM에 반영함
                vTaskDelay(dwellDelay);                                                 // 끝점에서 dwell만큼 멈춰 체감 속도를 조절함
                continue;                                                               // 끝점 처리 후 다음 루프로 넘어감
            }
            currentAngleDeg = (uint8_t)next;                                            // 상한 전이면 증가된 각도를 적용함
        }
        else
        {
            if (currentAngleDeg <= stepSweepDeg)                                        // 하강 방향에서 0도 근처면 스냅 처리함
            {
                currentAngleDeg = 0U;                                                   // 0도에 정확히 도달시키기 위해 스냅함
                goingUp = true;                                                         // 다음 루프부터 상승 방향으로 바꿈
                servoWriteAngle(currentAngleDeg);                                       // 0도 각도를 PWM에 반영함
                vTaskDelay(dwellDelay);                                                 // 끝점에서 dwell만큼 멈춰 체감 속도를 조절함
                continue;                                                               // 끝점 처리 후 다음 루프로 넘어감
            }
            currentAngleDeg = (uint8_t)(currentAngleDeg - stepSweepDeg);                // 0도 전이면 step만큼 감소시킴
        }

        servoWriteAngle(currentAngleDeg);                                               // 계산된 현재 각도를 PWM에 반영함
        vTaskDelay(kFrameDelay);                                                        // 프레임 주기만큼 쉬며 일정한 주기로 갱신되게 함
    }
}

void setup()
{
    Serial.begin(115200);                                                               // 시리얼 모니터 출력을 위해 UART를 초기화함
    delay(1500);                                                                        // USB/시리얼 연결 안정화를 위해 약간 대기함
    Serial.println("\nBOOT");                                                           // 부팅 직후 동작 확인을 위한 로그를 출력함

    servoInit();                                                                        // 서보 PWM(LEDC)을 초기화함
    servoWriteAngle(0);                                                                 // 시작 시 서보를 0도로 맞춰 안전 상태로 둠

    if (!canInit())                                                                     // CAN 초기화에 실패하면 안전을 위해 정지함
    {
        Serial.println("CAN init failed");                                              // CAN 초기화 실패를 로그로 남김
        while (true) { delay(1000); }                                                   // 더 진행하면 의미가 없으므로 무한 대기함
    }

    g_speedLevelQueue = xQueueCreate(4, sizeof(uint8_t));                               // speed_level을 전달할 큐를 생성함
    if (g_speedLevelQueue == NULL)                                                      // 큐 생성 실패 시 복구 불가이므로 정지함
    {
        Serial.println("Queue create failed");                                          // 큐 생성 실패를 로그로 남김
        while (true) { delay(1000); }                                                   // 더 진행하면 의미가 없으므로 무한 대기함
    }

    xTaskCreatePinnedToCore(canRxTask, "can_rx", 4096, NULL, 3, NULL, 1);               // CAN 수신 태스크를 코어1에 생성하고 우선순위 3으로 실행함
    xTaskCreatePinnedToCore(servoTask, "servo", 4096, NULL, 2, NULL, 1);                // 서보 제어 태스크를 코어1에 생성하고 우선순위 2로 실행함

    Serial.println("ESP32 Servo Node ready");                                           // 초기화 완료를 알리는 로그를 출력함
}

void loop()
{
    delay(1000);                                                                        // RTOS 태스크가 동작하므로 loop는 idle 용도로만 둠
}
