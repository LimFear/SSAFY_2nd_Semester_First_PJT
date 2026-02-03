#include <Arduino.h>
#include "driver/twai.h"

/* =========================
 * 핀 매핑 (CONTROL node)
 * ========================= */
#define CAN_RX_GPIO_PIN         GPIO_NUM_32
#define CAN_TX_GPIO_PIN         GPIO_NUM_33

#define SERVO_GPIO_PIN          GPIO_NUM_17

/* High-beam "출력" 핀 (릴레이/트랜지스터 등)
 * NOTE: GPIO27은 Right LED 추가핀으로 사용하므로 충돌 방지 위해 이동함.
 */
#define HIGH_OUTPUT_GPIO_PIN    GPIO_NUM_16

/* 상태/램프 핀 */
#define PIN_TURN_RIGHT          GPIO_NUM_18   /* Right (기존) */
#define PIN_TURN_RIGHT_EXTRA    GPIO_NUM_27   /* Right (추가) */

#define PIN_HIGH_INDICATOR_A    GPIO_NUM_19   /* High beam indicator */
#define PIN_HIGH_INDICATOR_B    GPIO_NUM_21   /* High beam indicator */

#define PIN_TURN_LEFT           GPIO_NUM_22   /* Left (기존) */
#define PIN_TURN_LEFT_EXTRA     GPIO_NUM_26   /* Left (추가) */

#define PIN_EMERGENCY_INDICATOR       GPIO_NUM_23   /* Emergency (기존) */
#define PIN_EMERGENCY_INDICATOR_EXTRA GPIO_NUM_25   /* Emergency (추가) */

/* =========================
 * 출력 논리 (Active-Low 기준)
 * - true  : LOW=ON, HIGH=OFF
 * - false : HIGH=ON, LOW=OFF
 * ========================= */
static const bool kOutputActiveLow = true;

static uint8_t outputLevelOn()
{
    if (kOutputActiveLow) {
        return LOW;
    }
    return HIGH;
}

static uint8_t outputLevelOff()
{
    if (kOutputActiveLow) {
        return HIGH;
    }
    return LOW;
}

static void writeOutput(gpio_num_t pin, bool on)
{
    if (on) {
        digitalWrite((int)pin, outputLevelOn());
        return;
    }
    digitalWrite((int)pin, outputLevelOff());
}

static void writeOutputPair(gpio_num_t pinA, gpio_num_t pinB, bool on)
{
    writeOutput(pinA, on);
    writeOutput(pinB, on);
}

/* =========================
 * CAN 프로토콜
 * ========================= */
#define CAN_ID_CMD_SERVO            0x200
#define SERVO_CMD_SET_SPEED_LEVEL   0x01

#define CAN_ID_CMD_HIGH_BEAM        0x210
#define HIGH_BEAM_CMD_SET_STATE     0x11

#define CAN_ID_CMD_TURN             0x220
#define TURN_CMD_PULSE              0x31   /* data[1]=dir */
#define TURN_CMD_HAZARD_SET         0x32   /* data[1]=0/1 */
#define TURN_DIR_LEFT               0x00
#define TURN_DIR_RIGHT              0x01

/* =========================
 * 속도 레벨
 * ========================= */
static const uint8_t kSpeedStop   = 0;
static const uint8_t kSpeedSlow   = 1;
static const uint8_t kSpeedNormal = 2;
static const uint8_t kSpeedFast   = 3;

/* =========================
 * 서보 설정
 * ========================= */
static const uint8_t  kServoMaxAngleDeg = 125;

static const int      kServoPwmFreqHz = 50;
static const int      kServoPwmResolutionBits = 16;

static const uint8_t  kSweepStepSlowDeg   = 3;
static const uint8_t  kSweepStepNormalDeg = 5;
static const uint8_t  kSweepStepFastDeg   = 8;

static const uint32_t kDwellSlowMs   = 120;
static const uint32_t kDwellNormalMs = 80;
static const uint32_t kDwellFastMs   = 40;

static const uint8_t  kReturnStepDeg = 10;
static const uint32_t kServoFrameDelayMs = 20;

/* =========================
 * Turn/Hazard 설정
 * ========================= */
static const uint32_t kTurnPulseMs = 500;
static const uint32_t kHazardToggleMs = 500;

/* =========================
 * 전역 상태
 * ========================= */
static volatile uint8_t g_targetServoSpeedLevel = kSpeedStop;
static volatile uint8_t g_targetHighBeamState = 0;

/* RX 디버그 카운터 */
static uint32_t g_rxCountTotal = 0;
static uint32_t g_rxCountServo = 0;
static uint32_t g_rxCountHighBeam = 0;
static uint32_t g_rxCountTurn = 0;
static uint32_t g_rxCountOther = 0;

/* =========================
 * 서보 스윕 상태
 * ========================= */
typedef struct
{
    uint8_t  currentAngleDeg;
    bool     goingUp;
    uint32_t nextStepMs;
    uint32_t dwellUntilMs;
    uint8_t  maxAngleDeg;
} ServoSweepState_t;

static ServoSweepState_t g_servo = { 0 };

/* =========================
 * Turn/Hazard 상태
 * ========================= */
static volatile bool g_hazardEnabled = false;
static uint32_t g_leftPulseUntilMs = 0;
static uint32_t g_rightPulseUntilMs = 0;

static bool g_hazardPhaseOn = false;
static uint32_t g_hazardNextToggleMs = 0;

static bool g_turnPhaseOn = false;

/* =========================
 * 서보 유틸
 * ========================= */
static uint32_t servoDutyFromPulseUs(uint32_t pulseUs)
{
    const uint32_t periodUs = 20000U;

    if (pulseUs > periodUs) {
        pulseUs = periodUs;
    }

    const uint32_t maxDuty = (1U << kServoPwmResolutionBits) - 1U;
    uint32_t duty = (pulseUs * maxDuty) / periodUs;
    return duty;
}

static uint32_t servoPulseUsFromAngle(uint8_t angleDeg)
{
    if (angleDeg > 180U) {
        angleDeg = 180U;
    }

    const uint32_t minUs = 500U;
    const uint32_t maxUs = 2500U;

    uint32_t pulse = minUs + (uint32_t)((maxUs - minUs) * angleDeg) / 180U;
    return pulse;
}

static void servoInit()
{
    bool ok = ledcAttach((int)SERVO_GPIO_PIN, kServoPwmFreqHz, kServoPwmResolutionBits);
    if (ok == false) {
        Serial.println("LEDC attach failed");
    }
}

static void servoWriteAngle(uint8_t angleDeg)
{
    uint32_t pulseUs = servoPulseUsFromAngle(angleDeg);
    uint32_t duty = servoDutyFromPulseUs(pulseUs);

    ledcWrite((int)SERVO_GPIO_PIN, duty);
}

static uint8_t sweepStepDegFromSpeed(uint8_t speedLevel)
{
    if (speedLevel == kSpeedSlow) {
        return kSweepStepSlowDeg;
    }
    if (speedLevel == kSpeedNormal) {
        return kSweepStepNormalDeg;
    }
    if (speedLevel == kSpeedFast) {
        return kSweepStepFastDeg;
    }
    return kSweepStepNormalDeg;
}

static uint32_t dwellDelayMsFromSpeed(uint8_t speedLevel)
{
    if (speedLevel == kSpeedSlow) {
        return kDwellSlowMs;
    }
    if (speedLevel == kSpeedNormal) {
        return kDwellNormalMs;
    }
    if (speedLevel == kSpeedFast) {
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
    g_servo.maxAngleDeg = kServoMaxAngleDeg;

    servoWriteAngle(g_servo.currentAngleDeg);
}

static void servoUpdate(uint32_t nowMs)
{
    if (nowMs < g_servo.dwellUntilMs) {
        return;
    }

    if (nowMs < g_servo.nextStepMs) {
        return;
    }

    g_servo.nextStepMs = nowMs + kServoFrameDelayMs;

    uint8_t speedLevel = g_targetServoSpeedLevel;

    if (speedLevel == kSpeedStop)
    {
        if (g_servo.currentAngleDeg == 0U) {
            return;
        }

        if (g_servo.currentAngleDeg > kReturnStepDeg) {
            g_servo.currentAngleDeg = (uint8_t)(g_servo.currentAngleDeg - kReturnStepDeg);
        }
        else {
            g_servo.currentAngleDeg = 0U;
        }

        servoWriteAngle(g_servo.currentAngleDeg);
        return;
    }

    uint8_t stepDeg = sweepStepDegFromSpeed(speedLevel);
    uint32_t dwellMs = dwellDelayMsFromSpeed(speedLevel);

    if (g_servo.goingUp)
    {
        uint16_t nextAngle = (uint16_t)g_servo.currentAngleDeg + (uint16_t)stepDeg;
        if (nextAngle >= g_servo.maxAngleDeg)
        {
            g_servo.currentAngleDeg = g_servo.maxAngleDeg;
            g_servo.goingUp = false;
            servoWriteAngle(g_servo.currentAngleDeg);

            g_servo.dwellUntilMs = nowMs + dwellMs;
            return;
        }

        g_servo.currentAngleDeg = (uint8_t)nextAngle;
        servoWriteAngle(g_servo.currentAngleDeg);
        return;
    }

    /* going down */
    if (g_servo.currentAngleDeg <= stepDeg)
    {
        g_servo.currentAngleDeg = 0U;
        g_servo.goingUp = true;
        servoWriteAngle(g_servo.currentAngleDeg);

        g_servo.dwellUntilMs = nowMs + dwellMs;
        return;
    }

    g_servo.currentAngleDeg = (uint8_t)(g_servo.currentAngleDeg - stepDeg);
    servoWriteAngle(g_servo.currentAngleDeg);
}

/* =========================
 * 램프/표시 초기화
 * ========================= */
static void lampsInit()
{
    pinMode((int)PIN_TURN_RIGHT, OUTPUT);
    pinMode((int)PIN_TURN_RIGHT_EXTRA, OUTPUT);

    pinMode((int)PIN_TURN_LEFT, OUTPUT);
    pinMode((int)PIN_TURN_LEFT_EXTRA, OUTPUT);

    pinMode((int)PIN_EMERGENCY_INDICATOR, OUTPUT);
    pinMode((int)PIN_EMERGENCY_INDICATOR_EXTRA, OUTPUT);

    pinMode((int)PIN_HIGH_INDICATOR_A, OUTPUT);
    pinMode((int)PIN_HIGH_INDICATOR_B, OUTPUT);

    writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, false);
    writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, false);
    writeOutputPair(PIN_EMERGENCY_INDICATOR, PIN_EMERGENCY_INDICATOR_EXTRA, false);

    writeOutput(PIN_HIGH_INDICATOR_A, false);
    writeOutput(PIN_HIGH_INDICATOR_B, false);
}

static void applyHighBeamIndicators(uint8_t highBeamState)
{
    bool on = (highBeamState != 0U);
    writeOutput(PIN_HIGH_INDICATOR_A, on);
    writeOutput(PIN_HIGH_INDICATOR_B, on);
}

static void applyHighBeamOutput(uint8_t highBeamState)
{
    bool on = (highBeamState != 0U);
    writeOutput(HIGH_OUTPUT_GPIO_PIN, on);
}

/* =========================
 * Turn/Hazard 로직
 * ========================= */
static void setHazardEnabled(bool enabled, uint32_t nowMs)
{
    g_hazardEnabled = enabled;

    g_leftPulseUntilMs = 0;
    g_rightPulseUntilMs = 0;
    g_turnPhaseOn = false;

    if (enabled) {
        g_hazardPhaseOn = true;
        g_hazardNextToggleMs = nowMs + kHazardToggleMs;

        writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, true);
        writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, true);
        writeOutputPair(PIN_EMERGENCY_INDICATOR, PIN_EMERGENCY_INDICATOR_EXTRA, true);
        return;
    }

    g_hazardPhaseOn = false;

    writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, false);
    writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, false);
    writeOutputPair(PIN_EMERGENCY_INDICATOR, PIN_EMERGENCY_INDICATOR_EXTRA, false);
}

static void startLeftPulse(uint32_t nowMs)
{
    if (g_hazardEnabled) {
        return;
    }

    /* 같은 방향을 또 누르면 OFF */
    if (g_leftPulseUntilMs != 0U) {
        g_leftPulseUntilMs = 0U;
        g_turnPhaseOn = false;
        writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, false);
        return;
    }

    /* 좌 시작: 우는 끄고 좌 점멸 시작 */
    g_rightPulseUntilMs = 0U;
    writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, false);

    g_turnPhaseOn = true;
    g_leftPulseUntilMs = nowMs + kTurnPulseMs;
    writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, true);
}

static void startRightPulse(uint32_t nowMs)
{
    if (g_hazardEnabled) {
        return;
    }

    /* 같은 방향을 또 누르면 OFF */
    if (g_rightPulseUntilMs != 0U) {
        g_rightPulseUntilMs = 0U;
        g_turnPhaseOn = false;
        writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, false);
        return;
    }

    /* 우 시작: 좌는 끄고 우 점멸 시작 */
    g_leftPulseUntilMs = 0U;
    writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, false);

    g_turnPhaseOn = true;
    g_rightPulseUntilMs = nowMs + kTurnPulseMs;
    writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, true);
}

static void signalsUpdate(uint32_t nowMs)
{
    if (g_hazardEnabled)
    {
        if (nowMs >= g_hazardNextToggleMs) {
            g_hazardNextToggleMs = nowMs + kHazardToggleMs;
            g_hazardPhaseOn = !g_hazardPhaseOn;

            writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, g_hazardPhaseOn);
            writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, g_hazardPhaseOn);
            writeOutputPair(PIN_EMERGENCY_INDICATOR, PIN_EMERGENCY_INDICATOR_EXTRA, g_hazardPhaseOn);
        }
        return;
    }

    if (g_leftPulseUntilMs != 0U || g_rightPulseUntilMs != 0U)
    {
        uint32_t* nextToggleMs = (g_leftPulseUntilMs != 0U) ? &g_leftPulseUntilMs : &g_rightPulseUntilMs;

        if (nowMs >= *nextToggleMs) {
            *nextToggleMs = nowMs + kTurnPulseMs;
            g_turnPhaseOn = !g_turnPhaseOn;
        }

        if (g_leftPulseUntilMs != 0U) {
            writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, g_turnPhaseOn);
            writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, false);
        } else {
            writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, g_turnPhaseOn);
            writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, false);
        }

        writeOutputPair(PIN_EMERGENCY_INDICATOR, PIN_EMERGENCY_INDICATOR_EXTRA, false);
        return;
    }

    /* 아무 것도 아니면 전부 OFF */
    writeOutputPair(PIN_TURN_LEFT, PIN_TURN_LEFT_EXTRA, false);
    writeOutputPair(PIN_TURN_RIGHT, PIN_TURN_RIGHT_EXTRA, false);
    writeOutputPair(PIN_EMERGENCY_INDICATOR, PIN_EMERGENCY_INDICATOR_EXTRA, false);
}

/* =========================
 * CAN init / poll
 * ========================= */
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

        g_rxCountTotal++;

        if (rxMessage.identifier == CAN_ID_CMD_SERVO)
        {
            if (rxMessage.data_length_code < 2) {
                continue;
            }

            uint8_t command = rxMessage.data[0];
            uint8_t speedLevel = rxMessage.data[1];

            if (command != SERVO_CMD_SET_SPEED_LEVEL) {
                continue;
            }

            if (speedLevel > kSpeedFast) {
                speedLevel = kSpeedFast;
            }

            g_targetServoSpeedLevel = speedLevel;

            g_rxCountServo++;
            Serial.printf("[RX] SERVO speed_level=%u\n", (unsigned)g_targetServoSpeedLevel);
            continue;
        }

        if (rxMessage.identifier == CAN_ID_CMD_HIGH_BEAM)
        {
            if (rxMessage.data_length_code < 2) {
                continue;
            }

            uint8_t command = rxMessage.data[0];
            uint8_t highState = rxMessage.data[1];

            if (command != HIGH_BEAM_CMD_SET_STATE) {
                continue;
            }

            g_targetHighBeamState = (highState != 0U) ? 1U : 0U;

            g_rxCountHighBeam++;
            Serial.printf("[RX] HIGH_BEAM state=%u\n", (unsigned)g_targetHighBeamState);
            continue;
        }

        if (rxMessage.identifier == CAN_ID_CMD_TURN)
        {
            if (rxMessage.data_length_code < 2) {
                continue;
            }

            uint8_t command = rxMessage.data[0];
            uint8_t value = rxMessage.data[1];
            uint32_t nowMs = millis();

            if (command == TURN_CMD_PULSE) {
                if (value == TURN_DIR_LEFT) {
                    startLeftPulse(nowMs);
                    Serial.println("[RX] TURN left pulse");
                    g_rxCountTurn++;
                }
                else if (value == TURN_DIR_RIGHT) {
                    startRightPulse(nowMs);
                    Serial.println("[RX] TURN right pulse");
                    g_rxCountTurn++;
                }
                continue;
            }

            if (command == TURN_CMD_HAZARD_SET) {
                bool enabled = (value != 0U);
                setHazardEnabled(enabled, nowMs);
                Serial.printf("[RX] HAZARD enabled=%u\n", enabled ? 1 : 0);
                g_rxCountTurn++;
                continue;
            }

            g_rxCountOther++;
            continue;
        }

        g_rxCountOther++;
    }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\nBOOT (CONTROL node)");

    pinMode((int)HIGH_OUTPUT_GPIO_PIN, OUTPUT);
    writeOutput(HIGH_OUTPUT_GPIO_PIN, false);

    lampsInit();

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

    applyHighBeamIndicators(g_targetHighBeamState);
    applyHighBeamOutput(g_targetHighBeamState);

    uint32_t nowMs = millis();

    static uint32_t lastStatMs = 0;
    if ((nowMs - lastStatMs) >= 1000) {
        lastStatMs = nowMs;
        Serial.printf("[RX-STAT] total=%lu servo=%lu high_beam=%lu turn=%lu other=%lu\n",
            (unsigned long)g_rxCountTotal,
            (unsigned long)g_rxCountServo,
            (unsigned long)g_rxCountHighBeam,
            (unsigned long)g_rxCountTurn,
            (unsigned long)g_rxCountOther);

        g_rxCountTotal = 0;
        g_rxCountServo = 0;
        g_rxCountHighBeam = 0;
        g_rxCountTurn = 0;
        g_rxCountOther = 0;
    }

    servoUpdate(nowMs);
    signalsUpdate(nowMs);

    delay(5);
}
