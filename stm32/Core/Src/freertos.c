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

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define EV_CAN_READY      0x00000001U
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
static osMessageQueueId_t qCanRx;
static osMessageQueueId_t qSensor;
static osMessageQueueId_t qServoCmd;

static osThreadId_t canRxTaskHandle;
static osThreadId_t controlTaskHandle;
static osThreadId_t canTxTaskHandle;

static osEventFlagsId_t evSys;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
static void CanRxTask(void *argument);
static void ControlTask(void *argument);
static void CanTxTask(void *argument);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */

  qCanRx    = osMessageQueueNew(32, sizeof(CanRawFrame_t), NULL);
  qSensor   = osMessageQueueNew(8,  sizeof(SensorSample_t), NULL);
  qServoCmd = osMessageQueueNew(8,  sizeof(ServoCommand_t), NULL);

  /* CAN ISR에서 넣을 큐 등록 */
  SensorControl_SetRxQueue(qCanRx);

  evSys = osEventFlagsNew(NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  const osThreadAttr_t canRxAttr = {
      .name = "can_rx",
      .priority = osPriorityHigh,
      .stack_size = 1024
  };

  const osThreadAttr_t controlAttr = {
      .name = "control",
      .priority = osPriorityAboveNormal,
      .stack_size = 1024
  };

  const osThreadAttr_t canTxAttr = {
      .name = "can_tx",
      .priority = osPriorityNormal,
      .stack_size = 1024
  };

  canRxTaskHandle = osThreadNew(CanRxTask, NULL, &canRxAttr);
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlAttr);
  canTxTaskHandle = osThreadNew(CanTxTask, NULL, &canTxAttr);

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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/**
  * @brief  CAN RX task
  *         - 여기서 CAN Start + Notification 활성화(커널 시작 후 실행되므로 안전)
  *         - ISR이 qCanRx에 넣은 RawFrame을 SensorSample로 변환해 qSensor에 넣음
  */
static void CanRxTask(void *argument)
{
  (void)argument;

  /* RTOS 태스크 시작 여부를 UART 로그로 확인하기 위한 출력임 */
  printf("CAN RX task start\r\n");

  /* CAN 필터/인터럽트를 설정하고 CAN을 시작하는 초기화 호출임 */
  if (SensorControl_CAN_Start(&hcan1) != HAL_OK) {
    printf("CAN start failed\r\n");
    /* CAN이 죽었으면 이후 동작이 무의미하므로 무한 대기하는 방어 로직임 */
    for (;;) {
      osDelay(1000);
    }
  }

  /* CAN 준비 완료를 다른 태스크에 알리기 위해 이벤트 플래그를 세팅하는 호출임 */
  (void)osEventFlagsSet(evSys, EV_CAN_READY);

  for (;;)
  {
    /* ISR이 넣은 CAN Raw 프레임을 큐에서 꺼내기 위한 변수임 */
    CanRawFrame_t frame;

    /* CAN Raw 프레임이 들어올 때까지 블로킹으로 대기하는 호출임 */
    osStatus_t st = osMessageQueueGet(qCanRx, &frame, NULL, osWaitForever);
    if (st != osOK) {
      continue;
    }

    /* 센서용 CAN ID(ESP32#1)가 아니면 무시하여 불필요한 파싱을 줄이는 조건임 */
    if (frame.std_id != CAN_ID_SENSOR_DHT) {
      continue;
    }

    /* 프로토콜 파싱 결과를 담기 위한 습도/온도 변수임 */
    float humidity = 0.0f;
    float temperature = 0.0f;

    /* data[0..3]의 x10 정수 값을 float(단위 %/°C)로 복원하는 파서 호출임 */
    int ok = protocol_parse_dht_x10(frame.data, frame.dlc, &humidity, &temperature);
    if (ok == 0) {
      continue;
    }

    /* 제어 태스크로 넘길 센서 샘플 구조체를 구성하는 코드임 */
    SensorSample_t sample;
    sample.humidity = humidity;
    sample.temperature = temperature;
    sample.tick_ms = (uint32_t)osKernelGetTickCount();

    /* 센서 샘플을 qSensor로 전달하여 제어 태스크가 판단할 수 있게 하는 호출임 */
    (void)osMessageQueuePut(qSensor, &sample, 0U, 0U);
  }
}

/**
  * @brief  Control task
  *         - 습도 구간(30/35/40)에 따라 서보 "속도 레벨"을 결정
  *         - 레벨 변경 시에만 송신 큐에 넣어 CAN 트래픽/로그를 줄임
  *         - 센서 타임아웃이면 STOP(0)으로 보내 0도로 복귀시키는 failsafe 수행
  *
  * @note   이 구현에서 cmd.angle_deg 필드는 "각도"가 아니라 "speed_level"로 사용함
  *         speed_level: 0=STOP, 1=SLOW(>=30), 2=NORMAL(>=35), 3=FAST(>=40)
  */
static void ControlTask(void *argument)
{
  (void)argument;

  /* RTOS 태스크 시작 여부를 UART 로그로 확인하기 위한 출력임 */
  printf("Control task start\r\n");

  /* 습도 기준 레벨 업 임계값(%)을 정의하는 상수임 */
  const float hum_on_30 = 30.0f;
  const float hum_on_35 = 35.0f;
  const float hum_on_40 = 40.0f;

  /* 레벨 다운 시 채터링 방지를 위한 히스테리시스(%)를 정의하는 상수임 */
  const float hysteresis = 0.5f;

  /* speed_level 정의(0=STOP, 1=SLOW, 2=NORMAL, 3=FAST)를 명확히 하기 위한 상수임 */
  const uint8_t SPEED_STOP   = 0;
  const uint8_t SPEED_SLOW   = 1; /* >=30 */
  const uint8_t SPEED_NORMAL = 2; /* >=35 (원래 속도) */
  const uint8_t SPEED_FAST   = 3; /* >=40 (가장 빠르되 약간 여유) */

  /* 현재 적용 중인 speed_level을 저장하여 변경 감지를 하기 위한 변수임 */
  uint8_t current_level = 255; /* invalid */
  /* 마지막으로 센서 샘플을 받은 시각을 기록하여 타임아웃을 판단하기 위한 변수임 */
  uint32_t last_sensor_tick = 0;

  for (;;)
  {
    /* 수신된 센서 샘플을 담기 위한 변수임 */
    SensorSample_t sample;

    /* 센서 샘플을 최대 1초 동안 기다리며 큐에서 꺼내는 호출임 */
    osStatus_t st = osMessageQueueGet(qSensor, &sample, NULL, 1000);

    if (st == osOK)
    {
      /* 타임아웃 판단을 위해 마지막 수신 시각을 갱신하는 코드임 */
      last_sensor_tick = sample.tick_ms;

      /* 현재 레벨을 기준으로 히스테리시스를 적용해 목표 레벨을 계산하는 변수임 */
      uint8_t target_level = current_level;

      /* 최초 진입 시에는 현재 습도에 맞는 레벨로 바로 초기화하는 분기임 */
      if (current_level == 255)
      {
        if (sample.humidity >= hum_on_40) {
          target_level = SPEED_FAST;
        } else if (sample.humidity >= hum_on_35) {
          target_level = SPEED_NORMAL;
        } else if (sample.humidity >= hum_on_30) {
          target_level = SPEED_SLOW;
        } else {
          target_level = SPEED_STOP;
        }
      }
      else
      {
        /* 레벨 업 조건은 즉시 반영하여 반응성을 확보하는 로직임 */
        if (sample.humidity >= hum_on_40) {
          target_level = SPEED_FAST;
        } else if (sample.humidity >= hum_on_35) {
          if (target_level < SPEED_NORMAL) {
            target_level = SPEED_NORMAL;
          }
        } else if (sample.humidity >= hum_on_30) {
          if (target_level < SPEED_SLOW) {
            target_level = SPEED_SLOW;
          }
        }

        /* 레벨 다운은 히스테리시스를 적용해 흔들림을 줄이는 로직임 */
        if (target_level == SPEED_FAST && sample.humidity < (hum_on_40 - hysteresis)) {
          target_level = SPEED_NORMAL;
        }
        if (target_level == SPEED_NORMAL && sample.humidity < (hum_on_35 - hysteresis)) {
          target_level = SPEED_SLOW;
        }
        if (target_level == SPEED_SLOW && sample.humidity < (hum_on_30 - hysteresis)) {
          target_level = SPEED_STOP;
        }
      }

      /* 레벨이 바뀌었을 때만 송신 큐로 넘겨 불필요한 송신을 줄이는 조건임 */
      if (target_level != current_level)
      {
        current_level = target_level;

        /* 송신 태스크로 넘길 명령 구조체를 구성하는 코드임(여기서는 speed_level을 사용함) */
        ServoCommand_t cmd;
        cmd.angle_deg = current_level; /* angle_deg 필드를 speed_level로 재해석함 */
        cmd.tick_ms = (uint32_t)osKernelGetTickCount();

        /* CAN 송신 전담 태스크가 처리하도록 큐에 명령을 넣는 호출임 */
        (void)osMessageQueuePut(qServoCmd, &cmd, 0U, 0U);

        /* 현재 습도와 적용된 속도 레벨을 로그로 확인하기 위한 출력임 */
        printf("Control: H=%.1f -> speed_level=%u\r\n",
               sample.humidity,
               (unsigned)current_level);
      }

      continue;
    }

    /* timeout 체크 */
    uint32_t now = (uint32_t)osKernelGetTickCount();
    uint32_t age = now - last_sensor_tick;

    /* 일정 시간(5초) 동안 센서가 안 들어오면 STOP으로 강제 복귀시키는 안전 로직임 */
    if (age > 5000U)
    {
      if (current_level != SPEED_STOP)
      {
        current_level = SPEED_STOP;

        /* STOP(0)을 송신 태스크에 전달해 ESP32가 0도로 복귀하도록 하는 명령임 */
        ServoCommand_t cmd;
        cmd.angle_deg = current_level; /* 0=STOP */
        cmd.tick_ms = now;

        (void)osMessageQueuePut(qServoCmd, &cmd, 0U, 0U);
        printf("Control: sensor timeout -> failsafe STOP\r\n");
      }
    }
  }
}

/**
  * @brief  CAN TX task
  *         - ServoCommand를 CAN 0x200으로 송신
  *         - 송신은 이 task만 수행(충돌 방지)
  *
  * @note   txData[1]은 "각도"가 아니라 "speed_level(0~3)"로 사용함
  */
static void CanTxTask(void *argument)
{
  (void)argument;

  /* RTOS 태스크 시작 여부를 UART 로그로 확인하기 위한 출력임 */
  printf("CAN TX task start\r\n");

  /* CAN 준비될 때까지 대기하여 초기화 순서 문제를 피하는 호출임 */
  (void)osEventFlagsWait(evSys, EV_CAN_READY, osFlagsWaitAny, osWaitForever);

  for (;;)
  {
    /* 제어 태스크가 만든 명령을 꺼내기 위한 구조체 변수임 */
    ServoCommand_t cmd;

    /* 송신할 명령이 들어올 때까지 큐에서 블로킹 대기하는 호출임 */
    osStatus_t st = osMessageQueueGet(qServoCmd, &cmd, NULL, osWaitForever);
    if (st != osOK) {
      continue;
    }

    /* CAN 송신 헤더/데이터를 구성하기 위한 로컬 변수들임 */
    CAN_TxHeaderTypeDef txHeader;
    uint8_t txData[8];
    uint32_t txMailbox = 0;

    /* 송신 헤더/데이터를 0으로 초기화해 쓰레기 값 전송을 방지하는 코드임 */
    memset(&txHeader, 0, sizeof(txHeader));
    memset(txData, 0, sizeof(txData));

    /* 표준 ID 0x200으로 데이터 프레임을 보내도록 설정하는 코드임 */
    txHeader.StdId = CAN_ID_CMD_SERVO;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 2;

    /* 첫 바이트는 명령 종류(기존 명령 상수 재사용)로 넣는 코드임 */
    txData[0] = SERVO_CMD_SET_ANGLE;
    /* 두 번째 바이트는 speed_level(0~3)로 넣는 코드임 */
    txData[1] = cmd.angle_deg;

    /* 메일박스가 없으면 잠깐 대기하여 송신 실패 확률을 낮추는 루프임 */
    uint32_t start = (uint32_t)osKernelGetTickCount();
    for (;;)
    {
      if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) > 0U) {
        break;
      }

      uint32_t now = (uint32_t)osKernelGetTickCount();
      if ((now - start) > 50U) {
        break;
      }
      osDelay(1);
    }

    /* CAN 프레임 송신을 수행하고 실패 시 로그를 남기는 코드임 */
    if (HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &txMailbox) != HAL_OK)
    {
      printf("CAN TX fail\r\n");
    }
  }
}


/* USER CODE END Application */

