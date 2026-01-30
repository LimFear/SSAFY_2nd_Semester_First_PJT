#include "decision.h"

static int16_t read_le_i16(const uint8_t* data)
{
    int16_t value = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    return value;
}

int protocol_parse_dht_x10(const uint8_t* data, uint8_t dlc, float* outHumidity, float* outTemperature)
{
    if (data == NULL) {
        return 0;
    }

    if (outHumidity == NULL) {
        return 0;
    }

    if (outTemperature == NULL) {
        return 0;
    }

    if (dlc < 4U) {
        return 0;
    }

    int16_t humidity_x10 = read_le_i16(&data[0]);
    int16_t temperature_x10 = read_le_i16(&data[2]);

    *outHumidity = ((float)humidity_x10) / 10.0f;
    *outTemperature = ((float)temperature_x10) / 10.0f;
    return 1;
}
