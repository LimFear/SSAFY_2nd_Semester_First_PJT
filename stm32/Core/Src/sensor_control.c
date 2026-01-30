#include "sensor_control.h"
#include "decision.h"
#include <string.h>

static CAN_HandleTypeDef *g_canHandle = NULL;
static osMessageQueueId_t g_rxQueue = NULL;

static void SensorControl_ConfigFilter_AcceptAll(CAN_HandleTypeDef *hcan)
{
  CAN_FilterTypeDef filter;
  memset(&filter, 0, sizeof(filter));

  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;

  /* Accept all standard IDs */
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow  = 0x0000;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow  = 0x0000;

  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  (void)HAL_CAN_ConfigFilter(hcan, &filter);
}

void SensorControl_SetRxQueue(osMessageQueueId_t rxQueue)
{
  g_rxQueue = rxQueue;
}

HAL_StatusTypeDef SensorControl_CAN_Start(CAN_HandleTypeDef *hcan)
{
  if (hcan == NULL) {
    return HAL_ERROR;
  }

  g_canHandle = hcan;
  SensorControl_ConfigFilter_AcceptAll(hcan);

  if (HAL_CAN_Start(hcan) != HAL_OK) {
    return HAL_ERROR;
  }

  if (HAL_CAN_ActivateNotification(hcan,
      CAN_IT_RX_FIFO0_MSG_PENDING |
      CAN_IT_ERROR |
      CAN_IT_BUSOFF |
      CAN_IT_LAST_ERROR_CODE) != HAL_OK) {
    return HAL_ERROR;
  }

  return HAL_OK;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (g_canHandle == NULL) {
    return;
  }
  if (hcan != g_canHandle) {
    return;
  }
  if (g_rxQueue == NULL) {
    return;
  }

  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0U)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
      break;
    }

    if (rxHeader.IDE != CAN_ID_STD) {
      continue;
    }

    CanRawFrame_t frame;
    frame.std_id = (uint16_t)rxHeader.StdId;
    frame.dlc = (uint8_t)rxHeader.DLC;

    memset(frame.data, 0, sizeof(frame.data));
    memcpy(frame.data, rxData, 8);

    (void)osMessageQueuePut(g_rxQueue, &frame, 0U, 0U);
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
  (void)hcan;
}
