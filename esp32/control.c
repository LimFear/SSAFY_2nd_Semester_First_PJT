#include <Arduino.h>
#include "driver/twai.h"

/* ===== CAN 핀 ===== */
#define CAN_RX_GPIO_PIN GPIO_NUM_32
#define CAN_TX_GPIO_PIN GPIO_NUM_33

/* ===== 서보 핀 ===== */
#define SERVO_GPIO_PIN  (GPIO_NUM_17)

/* ===== HIGH 출력 핀 ===== */
#define HIGH_GPIO_PIN   (GPIO_NUM_27)

/* ===== CAN 프로토콜 ===== */
#define CAN_ID_CMD_SERVO        0x200
#define SERVO_CMD_SET_ANGLE     0x01

#define CAN_ID_CMD_HIGH         0x210
#define HIGH_CMD_SET_STATE      0x11

/* ===== 속도 레벨 ===== */
static const uint8_t kSpeedLevelStop = 0;
static const uint8_t kSpeedLevelSlow = 1;
static const uint8_t kSpeedLevelNormal = 2;
static const uint8_t kSpeedLevelFast = 3;

/* ===== 서보 최대 각도 ===== */
static const uint8_t kMaxAngleLimit = 125;

/* ===== LEDC(서보 PWM) 설정 ===== */
static const int kPwmFreqHz = 50;
static const int kPwmResolutionBits = 16;

/* ===== 서보 속도 튜닝 ===== */
static const uint8_t kStepSweepSlowDeg = 3;
static const uint8_t kStepSweepNormalDeg = 5;
static const uint8_t kStepSweepFastDeg = 8;

static const uint32_t kDwellSlowMs = 120;
static const uint32_t kDwellNormalMs = 80;
static const uint32_t kDwellFastMs = 40;

static const uint8_t kStepStopDeg = 10;
static const uint32_t kFrameDelayMs = 20;

/* ===== 현재 목표 출력 ===== */
static volatile uint8_t g_targetSpeedLevel = kSpeedLevelStop;
static volatile uint8_t g_targetHighState = 0;

/* ===== 서보 상태 ===== */
typedef struct
{
    uint8_t currentAngleDeg;
    bool goingUp;
    uint32_t nextStepMs;
    uint32_t dwellUntilMs;
    uint8_t maxAngleDeg;
} ServoSweepState_t;

static ServoSweepState_t g_servo = { 0 };

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

static uint32_t dwellDelayMsFromSpeedLevel(uint8_t speedLevel)
{
    if (speedLevel == kSpeedLevelSlow) {
        return kDwellSlowMs;
    }
    if (speedLevel == kSpeedLevelNormal) {
        return kDwellNormalMs;
    }
    if (speedLevel == kSpeedLevelFast) {
        return kDwellFastMs;
    }
    return kDwellNormalMs;
}

static void servoControllerInit()
{
    g_servo.currentAngleDeg = 0;
    g_servo.goingUp = true;
    g_servo.nextStepMs = 0;
    g_servo.dwellUntilMs = 0;
    g_servo.maxAngleDeg = kMaxAngleLimit;

    servoWriteAngle(g_servo.currentAngleDeg);
}

static void applyHighState(uint8_t highState)
{
    uint8_t newState = (highState != 0U) ? 1U : 0U;

    if (newState != 0U) {
        digitalWrite((int)HIGH_GPIO_PIN, HIGH);
        return;
    }

    digitalWrite((int)HIGH_GPIO_PIN, LOW);
}

static void servoUpdate(uint32_t nowMs)
{
    if (nowMs < g_servo.dwellUntilMs) {
        return;
    }

    if (nowMs < g_servo.nextStepMs) {
        return;
    }

    g_servo.nextStepMs = nowMs + kFrameDelayMs;

    uint8_t speedLevel = g_targetSpeedLevel;

    if (speedLevel == kSpeedLevelStop)
    {
        if (g_servo.currentAngleDeg == 0U) {
            return;
        }

        if (g_servo.currentAngleDeg > kStepStopDeg) {
            g_servo.currentAngleDeg = (uint8_t)(g_servo.currentAngleDeg - kStepStopDeg);
        }
        else {
            g_servo.currentAngleDeg = 0U;
        }

        servoWriteAngle(g_servo.currentAngleDeg);
        return;
    }

    uint8_t stepSweepDeg = stepDegFromSpeedLevel(speedLevel);
    uint32_t dwellMs = dwellDelayMsFromSpeedLevel(speedLevel);

    if (g_servo.goingUp)
    {
        uint16_t next = (uint16_t)g_servo.currentAngleDeg + (uint16_t)stepSweepDeg;
        if (next >= g_servo.maxAngleDeg)
        {
            g_servo.currentAngleDeg = g_servo.maxAngleDeg;
            g_servo.goingUp = false;
            servoWriteAngle(g_servo.currentAngleDeg);

            g_servo.dwellUntilMs = nowMs + dwellMs;
            return;
        }

        g_servo.currentAngleDeg = (uint8_t)next;
        servoWriteAngle(g_servo.currentAngleDeg);
        return;
    }

    /* going down */
    if (g_servo.currentAngleDeg <= stepSweepDeg)
    {
        g_servo.currentAngleDeg = 0U;
        g_servo.goingUp = true;
        servoWriteAngle(g_servo.currentAngleDeg);

        g_servo.dwellUntilMs = nowMs + dwellMs;
        return;
    }

    g_servo.currentAngleDeg = (uint8_t)(g_servo.currentAngleDeg - stepSweepDeg);
    servoWriteAngle(g_servo.currentAngleDeg);
}

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

    Serial.println("CAN start OK");
    return true;
}

static void canPollCommands()
{
    for (;;)
    {
        twai_message_t rxMessage;
        esp_err_t result = twai_receive(&rxMessage, 0);
        if (result != ESP_OK) {
            break;
        }

        if ((rxMessage.flags & TWAI_MSG_FLAG_EXTD) != 0) {
            continue;
        }

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

            g_targetSpeedLevel = speedLevel;

            Serial.printf("[RX] SERVO speed_level=%u\n", (unsigned)g_targetSpeedLevel);
            continue;
        }

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

            g_targetHighState = (highState != 0U) ? 1U : 0U;

            Serial.printf("[RX] HIGH state=%u\n", (unsigned)g_targetHighState);
            continue;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\nBOOT (CONTROL node)");

    pinMode((int)HIGH_GPIO_PIN, OUTPUT);
    digitalWrite((int)HIGH_GPIO_PIN, LOW);

    servoInit();
    servoControllerInit();

    if (canInit() == false) {
        Serial.println("CAN init failed");
        while (true) { delay(1000); }
    }

    Serial.println("ESP32 Control Node ready");
}

void loop()
{
    canPollCommands();

    applyHighState(g_targetHighState);

    uint32_t nowMs = millis();
    servoUpdate(nowMs);

    delay(5);
}
