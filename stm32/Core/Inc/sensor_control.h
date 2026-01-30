#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

void SensorControl_SetRxQueue(osMessageQueueId_t rxQueue);
HAL_StatusTypeDef SensorControl_CAN_Start(CAN_HandleTypeDef *hcan);

/* HAL 콜백(인터럽트)에서 호출됨 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan);
void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan);

#ifdef __cplusplus
}
#endif

#endif
