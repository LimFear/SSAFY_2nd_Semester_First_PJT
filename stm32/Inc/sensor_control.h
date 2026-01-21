#ifndef SENSOR_CONTROL_H
#define SENSOR_CONTROL_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

/*
 * 사용 전제
 * - MX_CAN1_Init() 등 CubeMX 초기화 완료
 * - CAN 트랜시버/배선/종단은 정상
 *
 * 사용 흐름(폴링)
 * 1) SensorControl_CAN_Init(&hcan1)
 * 2) while(1)에서 SensorControl_Task()를 계속 호출
 * 3) 필요 시 SensorControl_DHT_TryRead()로 새 데이터만 읽기
 */

bool SensorControl_CAN_Init(CAN_HandleTypeDef *canHandle);

/* 폴링 수신 처리: while(1)에서 계속 호출 */
void SensorControl_Task(void);

/* 새 데이터가 들어온 경우에만 true 반환(한 번 읽으면 new 플래그가 내려감) */
bool SensorControl_DHT_TryRead(float *humidity, float *temperature);

/* 마지막 값(유효/나이 기준 포함)으로 읽기: maxAgeMs=0이면 나이 체크 안 함 */
bool SensorControl_DHT_GetLatest(float *humidity, float *temperature, uint32_t maxAgeMs);

#endif
