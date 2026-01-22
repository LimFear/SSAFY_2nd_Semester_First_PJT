#pragma once
#include <stdint.h>

#define CAN_ID_SENSOR_DHT              (0x100U)
#define CAN_ID_CMD_SERVO               (0x200U)

#define SERVO_CMD_SET_ANGLE            (0x01U)

typedef struct
{
    uint16_t std_id;
    uint8_t dlc;
    uint8_t data[8];
} CanRawFrame_t;

typedef struct
{
    float humidity;
    float temperature;
    uint32_t tick_ms;
} SensorSample_t;

typedef struct
{
    uint8_t angle_deg;     // 0..180
    uint32_t tick_ms;
} ServoCommand_t;

static inline uint16_t u16_from_le(const uint8_t low, const uint8_t high)
{
    uint16_t value = (uint16_t)low;
    value |= (uint16_t)((uint16_t)high << 8);
    return value;
}

static inline int protocol_parse_dht_x10(const uint8_t *data, uint8_t dlc, float *humidity, float *temperature)
{
    if (data == 0) {
        return 0;
    }
    if (humidity == 0) {
        return 0;
    }
    if (temperature == 0) {
        return 0;
    }
    if (dlc < 4U) {
        return 0;
    }

    uint16_t humidity_x10 = u16_from_le(data[0], data[1]);
    uint16_t temperature_x10 = u16_from_le(data[2], data[3]);

    *humidity = ((float)humidity_x10) / 10.0f;
    *temperature = ((float)temperature_x10) / 10.0f;
    return 1;
}
