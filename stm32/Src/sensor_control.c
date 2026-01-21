#include "sensor_control.h"
#include "main.h"

/* ESP32가 보내는 DHT 프레임의 표준 ID(11bit) */
#define SENSOR_CAN_STD_ID_DHT        (0x100U)

/* FIFO 선택 */
#define SENSOR_CAN_FIFO              (CAN_RX_FIFO0)

/* Task 한 번에 처리할 최대 프레임 수 */
#define SENSOR_MAX_DRAIN_PER_TASK    (16U)

/* 내부 캐시 */
typedef struct
{
  float humidity;
  float temperature;
  uint32_t lastUpdateTickMs;
  bool isValid;
  bool hasNewData;
} SensorDhtCache;

static CAN_HandleTypeDef *g_canHandle = NULL;
static SensorDhtCache g_dhtCache;

/* 내부 함수 */
static void SensorControl_ConfigFilterAcceptAll(void);
static bool SensorControl_PollReceiveOnce(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[8]);
static void SensorControl_HandleRx(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t rxData[8]);
static bool SensorControl_IsDhtFrame(const CAN_RxHeaderTypeDef *rxHeader);
static bool SensorControl_ParseDhtLittleEndian(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t rxData[8], float *humidity, float *temperature);

bool SensorControl_CAN_Init(CAN_HandleTypeDef *canHandle)
{
  if (canHandle == NULL) {
    return false;
  }

  g_canHandle = canHandle;

  memset(&g_dhtCache, 0, sizeof(g_dhtCache));
  g_dhtCache.isValid = false;
  g_dhtCache.hasNewData = false;

  SensorControl_ConfigFilterAcceptAll();

  if (HAL_CAN_Start(g_canHandle) != HAL_OK) {
    return false;
  }

  return true;
}

void SensorControl_Task(void)
{
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  uint32_t processedCount = 0;

  if (g_canHandle == NULL) {
    return;
  }

  while (processedCount < SENSOR_MAX_DRAIN_PER_TASK)
  {
    bool received = SensorControl_PollReceiveOnce(&rxHeader, rxData);
    if (received == false) {
      break;
    }

    SensorControl_HandleRx(&rxHeader, rxData);
    processedCount++;
  }
}

bool SensorControl_DHT_TryRead(float *humidity, float *temperature)
{
  if (humidity == NULL) {
    return false;
  }
  if (temperature == NULL) {
    return false;
  }

  if (g_dhtCache.isValid == false) {
    return false;
  }
  if (g_dhtCache.hasNewData == false) {
    return false;
  }

  *humidity = g_dhtCache.humidity;
  *temperature = g_dhtCache.temperature;

  g_dhtCache.hasNewData = false;
  return true;
}

bool SensorControl_DHT_GetLatest(float *humidity, float *temperature, uint32_t maxAgeMs)
{
  if (humidity == NULL) {
    return false;
  }
  if (temperature == NULL) {
    return false;
  }

  if (g_dhtCache.isValid == false) {
    return false;
  }

  if (maxAgeMs != 0U) {
    uint32_t nowTick = HAL_GetTick();
    uint32_t ageMs = nowTick - g_dhtCache.lastUpdateTickMs;
    if (ageMs > maxAgeMs) {
      return false;
    }
  }

  *humidity = g_dhtCache.humidity;
  *temperature = g_dhtCache.temperature;
  return true;
}

/* ===== 내부 구현 ===== */

static void SensorControl_ConfigFilterAcceptAll(void)
{
  CAN_FilterTypeDef filterConfig;

  if (g_canHandle == NULL) {
    return;
  }

  memset(&filterConfig, 0, sizeof(filterConfig));

  filterConfig.FilterBank = 0;
  filterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
  filterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

  filterConfig.FilterIdHigh = 0x0000;
  filterConfig.FilterIdLow  = 0x0000;
  filterConfig.FilterMaskIdHigh = 0x0000;
  filterConfig.FilterMaskIdLow  = 0x0000;

  filterConfig.FilterFIFOAssignment = SENSOR_CAN_FIFO;
  filterConfig.FilterActivation = ENABLE;

  filterConfig.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(g_canHandle, &filterConfig) != HAL_OK) {
    Error_Handler();
  }
}

static bool SensorControl_PollReceiveOnce(CAN_RxHeaderTypeDef *rxHeader, uint8_t rxData[8])
{
  if (g_canHandle == NULL) {
    return false;
  }
  if (rxHeader == NULL) {
    return false;
  }
  if (rxData == NULL) {
    return false;
  }

  uint32_t fillLevel = HAL_CAN_GetRxFifoFillLevel(g_canHandle, SENSOR_CAN_FIFO);
  if (fillLevel == 0U) {
    return false;
  }

  if (HAL_CAN_GetRxMessage(g_canHandle, SENSOR_CAN_FIFO, rxHeader, rxData) != HAL_OK) {
    return false;
  }

  return true;
}

static void SensorControl_HandleRx(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t rxData[8])
{
  if (rxHeader == NULL || rxData == NULL) {
    return;
  }

  if (SensorControl_IsDhtFrame(rxHeader) == false) {
    return;
  }

  float humidity = 0.0f;
  float temperature = 0.0f;

  if (SensorControl_ParseDhtLittleEndian(rxHeader, rxData, &humidity, &temperature) == false) {
    return;
  }

  g_dhtCache.humidity = humidity;
  g_dhtCache.temperature = temperature;
  g_dhtCache.lastUpdateTickMs = HAL_GetTick();
  g_dhtCache.isValid = true;
  g_dhtCache.hasNewData = true;
}

static bool SensorControl_IsDhtFrame(const CAN_RxHeaderTypeDef *rxHeader)
{
  if (rxHeader == NULL) {
    return false;
  }

  if (rxHeader->IDE != CAN_ID_STD) {
    return false;
  }

  if (rxHeader->StdId != SENSOR_CAN_STD_ID_DHT) {
    return false;
  }

  return true;
}

/* ESP32 송신 포맷과 동일:
 * DLC=4, little-endian
 * data[0]=hum_L, data[1]=hum_H, data[2]=temp_L, data[3]=temp_H
 * humidity = hum_x10 / 10.0, temperature = temp_x10 / 10.0
 */
static bool SensorControl_ParseDhtLittleEndian(const CAN_RxHeaderTypeDef *rxHeader, const uint8_t rxData[8],
                                              float *humidity, float *temperature)
{
  if (rxHeader == NULL || rxData == NULL || humidity == NULL || temperature == NULL) {
    return false;
  }

  uint8_t dlc = rxHeader->DLC;

  if (dlc >= 4U) {
    uint16_t humidityX10 = (uint16_t)((((uint16_t)rxData[1]) << 8) | (uint16_t)rxData[0]);
    uint16_t temperatureX10 = (uint16_t)((((uint16_t)rxData[3]) << 8) | (uint16_t)rxData[2]);

    *humidity = ((float)humidityX10) / 10.0f;
    *temperature = ((float)temperatureX10) / 10.0f;
    return true;
  }

  if (dlc >= 2U) {
    *humidity = (float)rxData[0];
    *temperature = (float)rxData[1];
    return true;
  }

  return false;
}
