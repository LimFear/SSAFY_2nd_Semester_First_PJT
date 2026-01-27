#include "sensor_control.h"
#include <string.h>

static CAN_HandleTypeDef *g_canHandle = NULL;
static osMessageQueueId_t g_rxQueue = NULL;

static void SensorControl_ConfigFilter_OnlyDht(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef filter;

    memset(&filter, 0, sizeof(filter));

    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;

    /* STD ID는 상위로 5비트 시프트 */
    filter.FilterIdHigh = (uint16_t)(CAN_ID_SENSOR_DHT << 5);
    filter.FilterIdLow = 0x0000;

    /* 마스크: STDID 11비트 전체 비교 */
    filter.FilterMaskIdHigh = (uint16_t)(0x7FFU << 5);
    filter.FilterMaskIdLow = 0x0000;

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

    SensorControl_ConfigFilter_OnlyDht(hcan);

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

    /* ISR에서는 “프레임 꺼내서 큐에 넣기”만 */
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

        /* timeout=0 으로 ISR-safe */
        (void)osMessageQueuePut(g_rxQueue, &frame, 0U, 0U);
    }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    (void)hcan;
    /* 필요하면 에러 카운터만 증가시키고, 복구는 태스크에서 수행 */
}
