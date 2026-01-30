/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.c
  * @brief   This file provides code for the configuration of the SPI instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

#include "spi.h"

/* USER CODE BEGIN 0 */
#include <string.h>

#include "decision.h"
/* “무조건 콜백이 도는 상태”를 위해:
 * 1) SPI1 슬레이브 RX IT를 항상 걸어둠
 * 2) RxCplt에서 ASAP 재시작
 * 3) 큐가 꽉 차면 1개 버리고 최신값을 넣음(최신 override를 우선)
 */
static osMessageQueueId_t g_spiCtrlQueue = NULL;
static uint8_t g_spiRxBuf[SPI_FRAME_LEN];

static uint8_t spi_calc_crc_xor4(const uint8_t* frame)
{
  uint8_t crc = 0U;
  crc ^= frame[0];
  crc ^= frame[1];
  crc ^= frame[2];
  crc ^= frame[3];
  return crc;
}

void SpiCtrl_SetRxQueue(osMessageQueueId_t queue)
{
  g_spiCtrlQueue = queue;
}

HAL_StatusTypeDef SpiCtrl_StartRxIT(SPI_HandleTypeDef* hspi)
{
  if (hspi == NULL) {
    return HAL_ERROR;
  }

  return HAL_SPI_Receive_IT(hspi, g_spiRxBuf, SPI_FRAME_LEN);
}
/* USER CODE END 0 */

SPI_HandleTypeDef hspi1;

void MX_SPI1_Init(void)
{
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_SLAVE;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_HARD_INPUT;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
}

void HAL_SPI_MspInit(SPI_HandleTypeDef* spiHandle)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(spiHandle->Instance==SPI1)
  {
    __HAL_RCC_SPI1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**SPI1 GPIO Configuration
    PA4     ------> SPI1_NSS
    PA5     ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    */
    GPIO_InitStruct.Pin = ESP32_SPI_CS_Pin|ESP32_SPI_SCK_Pin|ESP32_SPI_MISO_Pin|ESP32_SPI_MOSI_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF5_SPI1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* SPI1 interrupt Init
     * FreeRTOS 사용 시: ISR에서 osMessageQueuePut을 쓰려면 우선순위가
     * configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 이상(숫자 크게)이어야 함.
     * 보통 5가 안전한 기본값.
     */
    HAL_NVIC_SetPriority(SPI1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
  }
}

void HAL_SPI_MspDeInit(SPI_HandleTypeDef* spiHandle)
{
  if(spiHandle->Instance==SPI1)
  {
    __HAL_RCC_SPI1_CLK_DISABLE();
    HAL_GPIO_DeInit(GPIOA, ESP32_SPI_CS_Pin|ESP32_SPI_SCK_Pin|ESP32_SPI_MISO_Pin|ESP32_SPI_MOSI_Pin);
    HAL_NVIC_DisableIRQ(SPI1_IRQn);
  }
}

/* USER CODE BEGIN 1 */

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == NULL) {
    return;
  }

  if (hspi->Instance != SPI1) {
    return;
  }

  /* 다음 프레임을 절대 놓치지 않도록 먼저 로컬 복사 + ASAP 재시작 */
  uint8_t local[SPI_FRAME_LEN];
  memcpy(local, g_spiRxBuf, SPI_FRAME_LEN);

  (void)HAL_SPI_Receive_IT(hspi, g_spiRxBuf, SPI_FRAME_LEN);

  if (g_spiCtrlQueue == NULL) {
    return;
  }

  /* 프레임 검증 */
  if (local[0] != SPI_FRAME_SOF) {
    return;
  }

  if (local[1] != SPI_FRAME_VER) {
    return;
  }

  if (local[5] != SPI_FRAME_EOF) {
    return;
  }

  uint8_t crc = spi_calc_crc_xor4(local);
  if (crc != local[4]) {
    return;
  }

  SpiControlMessage_t msg;
  memset(&msg, 0, sizeof(msg));
  msg.wiper_mode = local[2];
  msg.high_mode  = local[3];
  msg.tick_ms    = HAL_GetTick();
  memcpy(msg.raw, local, SPI_FRAME_LEN);

  /* 큐가 꽉 차면 1개 버리고 최신값을 넣음 */
  osStatus_t putStatus = osMessageQueuePut(g_spiCtrlQueue, &msg, 0U, 0U);
  if (putStatus != 0) {
    SpiControlMessage_t dump;
    (void)osMessageQueueGet(g_spiCtrlQueue, &dump, NULL, 0U);
    (void)osMessageQueuePut(g_spiCtrlQueue, &msg, 0U, 0U);
  }
}

void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hspi)
{
  if (hspi == NULL) {
    return;
  }

  if (hspi->Instance != SPI1) {
    return;
  }

  /* 에러 시 abort 후 즉시 재수신 걸기 */
  (void)HAL_SPI_Abort_IT(hspi);
  (void)HAL_SPI_Receive_IT(hspi, g_spiRxBuf, SPI_FRAME_LEN);
}

/* USER CODE END 1 */
