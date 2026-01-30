/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "main.h"
#include "can.h"
#include "usart.h"
#include "sensor_control.h"
#include "decision.h"
#include "spi.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EV_CAN_READY      (0x00000001U)

/* ?��?���?? ?�� ?���?? ?��?�� ?�� ?���?? AUTO?��?�� ?��?�� 복�? */
#define SENSOR_STALE_MS   (5000U)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osMessageQueueId_t qSpiCtrlHandle;
osMessageQueueId_t qControlCmdHandle;
osMessageQueueId_t qCanRxHandle;

static osMessageQueueId_t g_qCanRx    = NULL;
static osMessageQueueId_t g_qSensor   = NULL;
static osMessageQueueId_t g_qServoCmd = NULL;
static osMessageQueueId_t g_qHighCmd  = NULL;
static osMessageQueueId_t g_qSpiCtrl  = NULL;

static osEventFlagsId_t g_evSys = NULL;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for controlTask */
osThreadId_t controlTaskHandle;
const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 1024 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};
/* Definitions for qSpiCtrl */
osMessageQueueId_t qSpiCtrlHandle;
const osMessageQueueAttr_t qSpiCtrl_attributes = {
  .name = "qSpiCtrl"
};
/* Definitions for qControlCmd */
osMessageQueueId_t qControlCmdHandle;
const osMessageQueueAttr_t qControlCmd_attributes = {
  .name = "qControlCmd"
};
/* Definitions for qCanRx */
osMessageQueueId_t qCanRxHandle;
const osMessageQueueAttr_t qCanRx_attributes = {
  .name = "qCanRx"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void CanRxTask(void *argument);
static void CanTxTask(void *argument);

static uint8_t clamp_speed_level(uint8_t level);
static CtrlMode_t sanitize_mode(uint8_t rawMode, CtrlMode_t current);

static HAL_StatusTypeDef can_send_servo_speed(uint8_t speedLevel);
static HAL_StatusTypeDef can_send_high_on(uint8_t on);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ControlTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of qSpiCtrl */
  qSpiCtrlHandle = osMessageQueueNew (8, sizeof(SpiControlMessage_t), &qSpiCtrl_attributes);

  /* creation of qControlCmd */
  qControlCmdHandle = osMessageQueueNew (8, sizeof(uint32_t), &qControlCmd_attributes);

  /* creation of qCanRx */
  qCanRxHandle = osMessageQueueNew (32, sizeof(CanRawFrame_t), &qCanRx_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
  g_qCanRx    = qCanRxHandle;
  g_qSpiCtrl  = qSpiCtrlHandle;

  g_qSensor   = osMessageQueueNew(8, sizeof(SensorSample_t), NULL);
  g_qServoCmd = osMessageQueueNew(8, sizeof(ServoCommand_t), NULL);
  g_qHighCmd  = osMessageQueueNew(8, sizeof(HighCommand_t),  NULL);

  SensorControl_SetRxQueue(g_qCanRx);
  SpiCtrl_SetRxQueue(g_qSpiCtrl);

  g_evSys = osEventFlagsNew(NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of controlTask */
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  const osThreadAttr_t canRxAttr = {
      .name = "can_rx",
      .priority = (osPriority_t)osPriorityHigh,
      .stack_size = 1024 * 4
  };

  const osThreadAttr_t canTxAttr = {
      .name = "can_tx",
      .priority = (osPriority_t)osPriorityNormal,
      .stack_size = 1024 * 4
  };

  (void)osThreadNew(CanRxTask, NULL, &canRxAttr);
  (void)osThreadNew(CanTxTask, NULL, &canTxAttr);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  printf("Default task start\r\n");

  /* SPI 슬레이브는 “항상 RX IT가 걸려있어야” 마스터 전송을 놓치지 않습니다. */
  if (SpiCtrl_StartRxIT(&hspi1) != HAL_OK) {
    printf("SPI RX start FAIL\r\n");
  } else {
    printf("SPI RX start OK\r\n");
  }

  /* Infinite loop */
  for(;;)
  {
    osDelay(1000);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ControlTask */
/**
* @brief Function implementing the controlTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ControlTask */
void ControlTask(void *argument)
{
  /* USER CODE BEGIN ControlTask */
  /* ===== Control Task =====
   * - WIPER/HIGH 각각 AUTO/ON/OFF 모드
   * - AUTO면 humidity threshold 기반으로 기존 알고리즘 유지
   * - ON/OFF면 강제 override
   */
  (void)argument;

  printf("Control task start\r\n");

  /* ===== AUTO threshold (정환님 요구에 맞춰 여기에서만 조정) ===== */
  const float hum_on_30 = 30.0f;
  const float hum_on_35 = 35.0f;
  const float hum_on_40 = 40.0f;
  const float hysteresis = 0.5f;

  /* HIGH AUTO 기준(임시): 습도 35 이상이면 ON */
  const float high_on = 35.0f;
  const float high_hys = 0.5f;

  CtrlMode_t wiperMode = CTRL_MODE_AUTO;
  CtrlMode_t highMode  = CTRL_MODE_AUTO;

  uint32_t lastSensorTick = 0U;
  float lastHumidity = 0.0f;

  /* AUTO 계산 결과 */
  uint8_t autoSpeedLevel = SERVO_SPEED_STOP;
  uint8_t autoHighOn = 0U;
  bool autoSpeedValid = false;
  bool autoHighValid = false;

  /* 최종 송신값(중복 송신 방지) */
  uint8_t lastSentSpeedLevel = 0xFFU;
  uint8_t lastSentHighOn = 0xFFU;

  /* Infinite loop */
  for(;;)
  {
	/* 1) SPI에서 온 override 명령 처리 */
	SpiControlMessage_t spiMsg;
	while (osMessageQueueGet(g_qSpiCtrl, &spiMsg, NULL, 0U) == osOK)
	{
	  CtrlMode_t newWiper = sanitize_mode(spiMsg.wiper_mode, wiperMode);
	  CtrlMode_t newHigh  = sanitize_mode(spiMsg.high_mode,  highMode);

	  if (newWiper != wiperMode) {
		wiperMode = newWiper;
		autoSpeedValid = false;
		printf("SPI: WIPER mode=%u\r\n", (unsigned)wiperMode);
	  }

	  if (newHigh != highMode) {
		highMode = newHigh;
		autoHighValid = false;
		printf("SPI: HIGH  mode=%u\r\n", (unsigned)highMode);
	  }
	}

	/* 2) 센서 샘플 갱신(없으면 100ms 동안 대기) */
	SensorSample_t sample;
	if (osMessageQueueGet(g_qSensor, &sample, NULL, 100U) == osOK)
	{
	  lastSensorTick = sample.tick_ms;
	  lastHumidity = sample.humidity;

	  /* WIPER AUTO 결정 */
	  if (wiperMode == CTRL_MODE_AUTO)
	  {
		if (!autoSpeedValid)
		{
		  if (lastHumidity >= hum_on_40) {
			autoSpeedLevel = SERVO_SPEED_FAST;
		  } else if (lastHumidity >= hum_on_35) {
			autoSpeedLevel = SERVO_SPEED_NORMAL;
		  } else if (lastHumidity >= hum_on_30) {
			autoSpeedLevel = SERVO_SPEED_SLOW;
		  } else {
			autoSpeedLevel = SERVO_SPEED_STOP;
		  }
		  autoSpeedValid = true;
		}
		else
		{
		  if (lastHumidity >= hum_on_40) {
			autoSpeedLevel = SERVO_SPEED_FAST;
		  } else if (lastHumidity >= hum_on_35) {
			if (autoSpeedLevel < SERVO_SPEED_NORMAL) {
			  autoSpeedLevel = SERVO_SPEED_NORMAL;
			}
		  } else if (lastHumidity >= hum_on_30) {
			if (autoSpeedLevel < SERVO_SPEED_SLOW) {
			  autoSpeedLevel = SERVO_SPEED_SLOW;
			}
		  }

		  if (autoSpeedLevel == SERVO_SPEED_FAST && lastHumidity < (hum_on_40 - hysteresis)) {
			autoSpeedLevel = SERVO_SPEED_NORMAL;
		  }
		  if (autoSpeedLevel == SERVO_SPEED_NORMAL && lastHumidity < (hum_on_35 - hysteresis)) {
			autoSpeedLevel = SERVO_SPEED_SLOW;
		  }
		  if (autoSpeedLevel == SERVO_SPEED_SLOW && lastHumidity < (hum_on_30 - hysteresis)) {
			autoSpeedLevel = SERVO_SPEED_STOP;
		  }
		}
	  }

	  /* HIGH AUTO 결정 */
	  if (highMode == CTRL_MODE_AUTO)
	  {
		if (!autoHighValid)
		{
		  autoHighOn = (lastHumidity >= high_on) ? 1U : 0U;
		  autoHighValid = true;
		}
		else
		{
		  if (autoHighOn == 0U) {
			if (lastHumidity >= high_on) {
			  autoHighOn = 1U;
			}
		  } else {
			if (lastHumidity < (high_on - high_hys)) {
			  autoHighOn = 0U;
			}
		  }
		}
	  }
	}

	/* 3) 센서 stale 판단(AUTO에서 안전복귀) */
	uint32_t now = (uint32_t)osKernelGetTickCount();
	bool sensorStale = true;

	if (lastSensorTick != 0U) {
	  uint32_t age = now - lastSensorTick;
	  if (age <= SENSOR_STALE_MS) {
		sensorStale = false;
	  }
	}

	/* 4) 최종 목표값 산출(override 우선) */
	uint8_t targetSpeedLevel = lastSentSpeedLevel;
	uint8_t targetHighOn = lastSentHighOn;

	if (wiperMode == CTRL_MODE_OFF) {
	  targetSpeedLevel = SERVO_SPEED_STOP;
	} else if (wiperMode == CTRL_MODE_ON) {
	  targetSpeedLevel = SERVO_SPEED_NORMAL;
	} else {
	  if (sensorStale) {
		targetSpeedLevel = SERVO_SPEED_STOP;
	  } else {
		targetSpeedLevel = clamp_speed_level(autoSpeedLevel);
	  }
	}

	if (highMode == CTRL_MODE_OFF) {
	  targetHighOn = 0U;
	} else if (highMode == CTRL_MODE_ON) {
	  targetHighOn = 1U;
	} else {
	  if (sensorStale) {
		targetHighOn = 0U;
	  } else {
		targetHighOn = autoHighValid ? autoHighOn : 0U;
	  }
	}

	/* 5) 변경이 있을 때만 큐로 전달 */
	if (targetSpeedLevel != lastSentSpeedLevel)
	{
	  lastSentSpeedLevel = targetSpeedLevel;

	  ServoCommand_t cmd;
	  cmd.speed_level = targetSpeedLevel;
	  cmd.tick_ms = now;

	  (void)osMessageQueuePut(g_qServoCmd, &cmd, 0U, 0U);
	  printf("WIPER: H=%.1f mode=%u -> speed=%u\r\n",
			 (double)lastHumidity,
			 (unsigned)wiperMode,
			 (unsigned)targetSpeedLevel);
	}

	if (targetHighOn != lastSentHighOn)
	{
	  lastSentHighOn = targetHighOn;

	  HighCommand_t cmd;
	  cmd.on = targetHighOn;
	  cmd.tick_ms = now;

	  (void)osMessageQueuePut(g_qHighCmd, &cmd, 0U, 0U);
	  printf("HIGH : H=%.1f mode=%u -> on=%u\r\n",
			 (double)lastHumidity,
			 (unsigned)highMode,
			 (unsigned)targetHighOn);
	}
  }
  /* USER CODE END ControlTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
static uint8_t clamp_speed_level(uint8_t level)
{
  if (level > SERVO_SPEED_FAST) {
    return SERVO_SPEED_FAST;
  }
  return level;
}

static CtrlMode_t sanitize_mode(uint8_t rawMode, CtrlMode_t current)
{
  if (rawMode == SPI_MODE_NOCHANGE) {
    return current;
  }

  if (rawMode == (uint8_t)CTRL_MODE_AUTO) {
    return CTRL_MODE_AUTO;
  }

  if (rawMode == (uint8_t)CTRL_MODE_ON) {
    return CTRL_MODE_ON;
  }

  if (rawMode == (uint8_t)CTRL_MODE_OFF) {
    return CTRL_MODE_OFF;
  }

  return current;
}

/* ===== CAN RX Task ===== */
static void CanRxTask(void *argument)
{
  (void)argument;

  printf("CAN RX task start\r\n");

  if (SensorControl_CAN_Start(&hcan1) != HAL_OK) {
    printf("CAN start failed\r\n");
    for (;;) {
      osDelay(1000);
    }
  }

  (void)osEventFlagsSet(g_evSys, EV_CAN_READY);

  for (;;)
  {
    CanRawFrame_t frame;
    osStatus_t st = osMessageQueueGet(g_qCanRx, &frame, NULL, osWaitForever);
    if (st != osOK) {
      continue;
    }

    if (frame.std_id != CAN_ID_SENSOR_DHT) {
      continue;
    }

    float humidity = 0.0f;
    float temperature = 0.0f;

    int ok = protocol_parse_dht_x10(frame.data, frame.dlc, &humidity, &temperature);
    if (ok == 0) {
      continue;
    }

    SensorSample_t sample;
    sample.humidity = humidity;
    sample.temperature = temperature;
    sample.tick_ms = (uint32_t)osKernelGetTickCount();

    (void)osMessageQueuePut(g_qSensor, &sample, 0U, 0U);
  }
}



/* ===== CAN TX helpers ===== */
static HAL_StatusTypeDef can_send_servo_speed(uint8_t speedLevel)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t mailbox = 0U;

  memset(&txHeader, 0, sizeof(txHeader));
  memset(txData, 0, sizeof(txData));

  txHeader.StdId = CAN_ID_CMD_SERVO;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 2;

  txData[0] = SERVO_CMD_SET_ANGLE;
  txData[1] = clamp_speed_level(speedLevel);

  return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &mailbox);
}

static HAL_StatusTypeDef can_send_high_on(uint8_t on)
{
  CAN_TxHeaderTypeDef txHeader;
  uint8_t txData[8];
  uint32_t mailbox = 0U;

  memset(&txHeader, 0, sizeof(txHeader));
  memset(txData, 0, sizeof(txData));

  txHeader.StdId = CAN_ID_CMD_HIGH;
  txHeader.IDE = CAN_ID_STD;
  txHeader.RTR = CAN_RTR_DATA;
  txHeader.DLC = 2;

  txData[0] = HIGH_CMD_SET_STATE;
  txData[1] = (on != 0U) ? 1U : 0U;

  return HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &mailbox);
}

/* ===== CAN TX Task ===== */
static void CanTxTask(void *argument)
{
  (void)argument;

  printf("CAN TX task start\r\n");
  (void)osEventFlagsWait(g_evSys, EV_CAN_READY, osFlagsWaitAny, osWaitForever);

  for (;;)
  {
    ServoCommand_t servoCmd;
    if (osMessageQueueGet(g_qServoCmd, &servoCmd, NULL, 20U) == osOK)
    {
      if (can_send_servo_speed(servoCmd.speed_level) != HAL_OK) {
        printf("CAN TX servo FAIL\r\n");
      }
    }

    HighCommand_t highCmd;
    while (osMessageQueueGet(g_qHighCmd, &highCmd, NULL, 0U) == osOK)
    {
      if (can_send_high_on(highCmd.on) != HAL_OK) {
        printf("CAN TX high FAIL\r\n");
      }
    }
  }
}

/* USER CODE END Application */

