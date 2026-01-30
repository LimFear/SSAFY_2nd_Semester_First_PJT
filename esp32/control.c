#include <Arduino.h>
#include "driver/twai.h"

/* ===== CAN 핀(사용중인 핀으로 수정) ===== */
#define CAN_RX_GPIO_PIN GPIO_NUM_32
#define CAN_TX_GPIO_PIN GPIO_NUM_33

/* ===== 서보 핀(원하는 핀으로 수정) ===== */
#define SERVO_GPIO_PIN  (GPIO_NUM_17)

/* ===== HIGH 출력 핀(원하는 핀으로 수정) ===== */
#define HIGH_GPIO_PIN   (GPIO_NUM_27)   // 예: 릴레이/LED/트랜지스터 구동

/* ===== CAN 프로토콜 ===== */
#define CAN_ID_CMD_SERVO        0x200
#define SERVO_CMD_SET_ANGLE     0x01    // data[0]

/* HIGH 제어 프레임 추가 */
#define CAN_ID_CMD_HIGH         0x210
#define HIGH_CMD_SET_STATE      0x11    // data[0], data[1]=0/1

/*
  data[1] = speed_level
  0=STOP, 1=SLOW, 2=NORMAL, 3=FAST
*/

/* ===== 서보 최대 각도 ===== */
static const uint8_t kMaxAngleLimit = 125;

/* ===== LEDC(서보 PWM) 설정 ===== */
static const int kPwmFreqHz = 50;
static const int kPwmResolutionBits = 16;

/* ===== RTOS ===== */
static QueueHandle_t g_speedLevelQueue;

/* ===== 속도 레벨 상수 ===== */
static const uint8_t kSpeedLevelStop = 0;
static const uint8_t kSpeedLevelSlow = 1;
static const uint8_t kSpeedLevelNormal = 2;
static const uint8_t kSpeedLevelFast = 3;

/* ===== 서보 속도 튜닝 ===== */
static const uint8_t kStepSweepSlowDeg = 3;
static const uint8_t kStepSweepNormalDeg = 5;
static const uint8_t kStepSweepFastDeg = 8;

static const TickType_t kDwellSlow = pdMS_TO_TICKS(120);
static const TickType_t kDwellNormal = pdMS_TO_TICKS(80);
static const TickType_t kDwellFast = pdMS_TO_TICKS(40);

static const uint8_t kStepStopDeg = 10;
static const TickType_t kFrameDelay = pdMS_TO_TICKS(20);

/* 50Hz(20000us)에서 pulse_us를 duty로 변환 */
static uint32_t servoDutyFromPulseUs(uint32_t pulse_us)
{
    const uint32_t period_us = 20000U;

    if (pulse_us > period_us) {
        pulse_us = period_us;
    }

    const uint32_t maxDuty = (1U << kPwmResolutionBits) - 1U;
    uint32_t duty = (pulse_us * maxDuty) / period_us;
    return duty;
}

static uint32_t pulseUsFromAngle(uint8_t angle_deg)
{
    if (angle_deg > 180U) {
        angle_deg = 180U;
    }

    const uint32_t min_us = 500U;
    const uint32_t max_us = 2500U;

    uint32_t pulse = min_us + (uint32_t)((max_us - min_us) * angle_deg) / 180U;
    return pulse;
}

static void servoInit()
{
    bool ok = ledcAttach((int)SERVO_GPIO_PIN, kPwmFreqHz, kPwmResolutionBits);
    if (ok == false) {
        Serial.println("LEDC attach failed");
    }
}

static void servoWriteAngle(uint8_t angle_deg)
{
    uint32_t pulse = pulseUsFromAngle(angle_deg);
    uint32_t duty = servoDutyFromPulseUs(pulse);

    ledcWrite((int)SERVO_GPIO_PIN, duty);
}

static uint8_t stepDegFromSpeedLevel(uint8_t speedLevel)
{
    if (speedLevel == kSpeedLevelSlow) {
        return kStepSweepSlowDeg;
    }
    if (speedLevel == kSpeedLevelNormal) {
        return kStepSweepNormalDeg;
    }
    if (speedLevel == kSpeedLevelFast) {
        return kStepSweepFastDeg;
    }
    return kStepSweepNormalDeg;
}

static TickType_t dwellDelayFromSpeedLevel(uint8_t speedLevel)
{
    if (speedLevel == kSpeedLevelSlow) {
        return kDwellSlow;
    }
    if (speedLevel == kSpeedLevelNormal) {
        return kDwellNormal;
    }
    if (speedLevel == kSpeedLevelFast) {
        return kDwellFast;
    }
    return kDwellNormal;
}

/* ===== CAN 초기화 ===== */
static bool canInit()
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

    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("CAN start fail: %d\n", (int)result);
        return false;
    }

    return true;
}

/* ===== HIGH 출력 적용 ===== */
static void applyHighState(uint8_t highState)
{
    if (highState != 0U) {
        digitalWrite((int)HIGH_GPIO_PIN, HIGH);
        Serial.println("[APPLY] HIGH = ON");
        return;
    }

    digitalWrite((int)HIGH_GPIO_PIN, LOW);
    Serial.println("[APPLY] HIGH = OFF");
}

/* ===== CAN RX Task ===== */
static void canRxTask(void* param)
{
    (void)param;

    Serial.println("canRxTask start");

    for (;;)
    {
        twai_message_t rxMessage;
        esp_err_t result = twai_receive(&rxMessage, pdMS_TO_TICKS(1000));
        if (result != ESP_OK) {
            continue;
        }

        if ((rxMessage.flags & TWAI_MSG_FLAG_EXTD) != 0) {
            continue;
        }

        /* 1) 서보(speed_level) 프레임 */
        if (rxMessage.identifier == CAN_ID_CMD_SERVO)
        {
            if (rxMessage.data_length_code < 2) {
                continue;
            }

            uint8_t command = rxMessage.data[0];
            uint8_t speedLevel = rxMessage.data[1];

            if (command != SERVO_CMD_SET_ANGLE) {
                continue;
            }

            if (speedLevel > kSpeedLevelFast) {
                speedLevel = kSpeedLevelFast;
            }

            Serial.printf("[RX] SERVO id=0x%X cmd=%u speed_level=%u\n",
                (unsigned)rxMessage.identifier,
                (unsigned)command,
                (unsigned)speedLevel);

            if (uxQueueSpacesAvailable(g_speedLevelQueue) == 0)
            {
                uint8_t discarded;
                (void)xQueueReceive(g_speedLevelQueue, &discarded, 0);
            }

            (void)xQueueSend(g_speedLevelQueue, &speedLevel, 0);
            continue;
        }

        /* 2) HIGH(0/1) 프레임 추가 */
        if (rxMessage.identifier == CAN_ID_CMD_HIGH)
        {
            if (rxMessage.data_length_code < 2) {
                continue;
            }

            uint8_t command = rxMessage.data[0];
            uint8_t highState = rxMessage.data[1];

            if (command != HIGH_CMD_SET_STATE) {
                continue;
            }

            highState = (highState != 0U) ? 1U : 0U;

            Serial.printf("[RX] HIGH id=0x%X cmd=%u state=%u\n",
                (unsigned)rxMessage.identifier,
                (unsigned)command,
                (unsigned)highState);

            applyHighState(highState);
            continue;
        }
    }
}

/* ===== Servo Task ===== */
enum ServoMode
{
    SERVO_MODE_STOP = 0,
    SERVO_MODE_SWEEP = 1,
};

static void servoTask(void* param)
{
    (void)param;

    Serial.println("servoTask start");

    ServoMode mode = SERVO_MODE_STOP;

    uint8_t currentAngleDeg = 0;
    uint8_t maxAngleDeg = kMaxAngleLimit;
    bool goingUp = true;

    uint8_t currentSpeedLevel = kSpeedLevelStop;
    uint8_t stepSweepDeg = stepDegFromSpeedLevel(currentSpeedLevel);
    TickType_t dwellDelay = dwellDelayFromSpeedLevel(currentSpeedLevel);

    servoWriteAngle(currentAngleDeg);

    for (;;)
    {
        uint8_t newSpeedLevel;
        if (xQueueReceive(g_speedLevelQueue, &newSpeedLevel, 0) == pdTRUE)
        {
            if (newSpeedLevel == kSpeedLevelStop)
            {
                mode = SERVO_MODE_STOP;
                currentSpeedLevel = kSpeedLevelStop;
                stepSweepDeg = stepDegFromSpeedLevel(currentSpeedLevel);
                dwellDelay = dwellDelayFromSpeedLevel(currentSpeedLevel);
                Serial.println("[APPLY] SERVO STOP -> return to 0");
            }
            else
            {
                mode = SERVO_MODE_SWEEP;
                currentSpeedLevel = newSpeedLevel;
                stepSweepDeg = stepDegFromSpeedLevel(currentSpeedLevel);
                dwellDelay = dwellDelayFromSpeedLevel(currentSpeedLevel);
                goingUp = true;

                Serial.printf("[APPLY] SWEEP 0 <-> %u, level=%u, step=%u, dwell=%lu\n",
                    (unsigned)maxAngleDeg,
                    (unsigned)currentSpeedLevel,
                    (unsigned)stepSweepDeg,
                    (unsigned long)(dwellDelay * portTICK_PERIOD_MS));
            }
        }

        if (mode == SERVO_MODE_STOP)
        {
            if (currentAngleDeg == 0U)
            {
                vTaskDelay(kFrameDelay);
                continue;
            }

            if (currentAngleDeg > kStepStopDeg)
            {
                currentAngleDeg = (uint8_t)(currentAngleDeg - kStepStopDeg);
            }
            else
            {
                currentAngleDeg = 0U;
            }

            servoWriteAngle(currentAngleDeg);

            if (currentAngleDeg == 0U)
            {
                Serial.println("[DONE] servo at 0");
            }

            vTaskDelay(kFrameDelay);
            continue;
        }

        /* SERVO_MODE_SWEEP */
        if (goingUp)
        {
            uint16_t next = (uint16_t)currentAngleDeg + (uint16_t)stepSweepDeg;
            if (next >= maxAngleDeg)
            {
                currentAngleDeg = maxAngleDeg;
                goingUp = false;
                servoWriteAngle(currentAngleDeg);
                vTaskDelay(dwellDelay);
                continue;
            }
            currentAngleDeg = (uint8_t)next;
        }
        else
        {
            if (currentAngleDeg <= stepSweepDeg)
            {
                currentAngleDeg = 0U;
                goingUp = true;
                servoWriteAngle(currentAngleDeg);
                vTaskDelay(dwellDelay);
                continue;
            }
            currentAngleDeg = (uint8_t)(currentAngleDeg - stepSweepDeg);
        }

        servoWriteAngle(currentAngleDeg);
        vTaskDelay(kFrameDelay);
    }
}

void setup()
{
    Serial.begin(115200);
    delay(1500);
    Serial.println("\nBOOT");

    /* HIGH 핀 초기화 */
    pinMode((int)HIGH_GPIO_PIN, OUTPUT);
    digitalWrite((int)HIGH_GPIO_PIN, LOW);

    servoInit();
    servoWriteAngle(0);

    if (canInit() == false)
    {
        Serial.println("CAN init failed");
        while (true) { delay(1000); }
    }

    g_speedLevelQueue = xQueueCreate(4, sizeof(uint8_t));
    if (g_speedLevelQueue == NULL)
    {
        Serial.println("Queue create failed");
        while (true) { delay(1000); }
    }

    xTaskCreatePinnedToCore(canRxTask, "can_rx", 4096, NULL, 3, NULL, 1);
    xTaskCreatePinnedToCore(servoTask, "servo", 4096, NULL, 2, NULL, 1);

    Serial.println("ESP32 Control Node ready");
}

void loop()
{
    delay(1000);
}
