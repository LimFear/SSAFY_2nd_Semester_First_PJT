#ifndef DECISION_H
#define DECISION_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ===== CAN IDs ===== */
#define CAN_ID_SENSOR_DHT      (0x100U)   /* ESP32(send) -> STM32 */
#define CAN_ID_CMD_SERVO       (0x200U)   /* STM32 -> ESP32(control) */

/* ===== Servo protocol ===== */
#define SERVO_CMD_SET_ANGLE    (0x01U)    /* data[0] */
#define SERVO_SPEED_STOP       (0U)
#define SERVO_SPEED_SLOW       (1U)
#define SERVO_SPEED_NORMAL     (2U)
#define SERVO_SPEED_FAST       (3U)

/* (선택) HIGH 채널을 CAN으로도 보낼 때 */
#define CAN_ID_CMD_HIGH        (0x210U)
#define HIGH_CMD_SET_STATE     (0x10U)

/* ===== Types ===== */
typedef struct
{
    uint16_t std_id;
    uint8_t  dlc;
    uint8_t  data[8];
} CanRawFrame_t;

typedef struct
{
    float    humidity;
    float    temperature;
    uint32_t tick_ms;
} SensorSample_t;

typedef struct
{
    uint8_t  speed_level;
    uint32_t tick_ms;
} ServoCommand_t;

typedef struct
{
    uint8_t  on;
    uint32_t tick_ms;
} HighCommand_t;

typedef enum
{
    CTRL_MODE_AUTO = 0,
    CTRL_MODE_ON   = 1,
    CTRL_MODE_OFF  = 2,
} CtrlMode_t;

/* ===== SPI control frame (ESP32 mqtt -> STM32) =====
 * [0]=SOF 0xA5
 * [1]=VER 0x01
 * [2]=WIPER_MODE (0=AUTO,1=ON,2=OFF, 0xFF=NOCHANGE)
 * [3]=HIGH_MODE  (0=AUTO,1=ON,2=OFF, 0xFF=NOCHANGE)
 * [4]=CRC XOR([0..3])
 * [5]=EOF 0x5A
 */
#define SPI_FRAME_SOF          (0xA5U)
#define SPI_FRAME_VER          (0x01U)
#define SPI_FRAME_EOF          (0x5AU)
#define SPI_FRAME_LEN          (6U)
#define SPI_MODE_NOCHANGE      (0xFFU)

typedef struct
{
    uint8_t  wiper_mode;
    uint8_t  high_mode;
    uint32_t tick_ms;
    uint8_t  raw[SPI_FRAME_LEN];
} SpiControlMessage_t;

/* ===== Helpers ===== */
/* ESP32(send)에서 humidity_x10, temperature_x10 (little-endian)로 보낸 데이터를 float로 복원 */
int protocol_parse_dht_x10(const uint8_t* data, uint8_t dlc, float* outHumidity, float* outTemperature);

#ifdef __cplusplus
}
#endif

#endif /* DECISION_H */
