/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* 
 * [SSAFY 2학기 공통PJT - 통합 게이트웨이 메인 로직 (FreeRTOS)]
 * 
 * 게이트웨이(STM32)는 "지휘본부" 역할을 해.
 * 1. 센서 데이터를 받아 검증하고(CanRx),
 * 2. 주변 환경(조도/습도)에 맞는 최적의 명령을 판단하며(Control),
 * 3. 액츄에이터에게 명령을 쏘고 답장을 기다려(CanTx).
 */

/* Private includes ----------------------------------------------------------*/
#include "can.h"
#include "usart.h"
#include "decision.h"
#include <stdio.h>
#include <string.h>

/* OS Queues: 태스크 간에 데이터를 주고받는 "통로"들이야 */
static osMessageQueueId_t qCanRx;      // CAN 하드웨어에서 받은 날것의 데이터를 담아
static osMessageQueueId_t qSensor;     // 해석이 완료된 센서 수치( Lux, %, °C)를 판단 태스크로 보낼 때 사용
static osMessageQueueId_t qControlCmd; // 판단 태스크가 내린 명령을 전송 태스크로 보낼 때 사용

/* OS Tasks: 동시에 일하는 3명의 일꾼이야 */
static osThreadId_t canRxTaskHandle;
static osThreadId_t controlTaskHandle;
static osThreadId_t canTxTaskHandle;

/* MX_FREERTOS_Init: 운영 인터페이스 초기화 (큐와 태크스를 만들어) */
void MX_FREERTOS_Init(void) {
  qCanRx      = osMessageQueueNew(32, sizeof(CanRawFrame_t), NULL);
  qSensor     = osMessageQueueNew(8,  sizeof(SensorSample_t), NULL);
  qControlCmd = osMessageQueueNew(8,  sizeof(ControlCommand_t), NULL);

  // 태스크 생성 (우선순위 설정이 중요해. 데이터 수신이 가장 급하니까 High!)
  const osThreadAttr_t canRxAttr = {.name = "can_rx", .priority = osPriorityHigh, .stack_size = 1024};
  const osThreadAttr_t controlAttr = {.name = "control", .priority = osPriorityAboveNormal, .stack_size = 1024};
  const osThreadAttr_t canTxAttr = {.name = "can_tx", .priority = osPriorityNormal, .stack_size = 1024};

  canRxTaskHandle = osThreadNew(CanRxTask, NULL, &canRxAttr);
  controlTaskHandle = osThreadNew(ControlTask, NULL, &controlAttr);
  canTxTaskHandle = osThreadNew(CanTxTask, NULL, &canTxAttr);
}

/* 
 * [일꾼 A] CanRxTask: CAN 데이터 수집 및 무결성 검증
 * ESP32 센서 노드가 보낸 데이터를 `decision.h`에 적힌 v2.2 규칙으로 꼼꼼히 검사해.
 */
static void CanRxTask(void *argument) {
  printf("Gateway CAN RX engine started (v2.2 Standard)\r\n");
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

  SensorSample_t sample = {0};

  for (;;) {
    CanRawFrame_t frame;
    // CAN 버스에서 메시지가 오기를 계속 기다려
    if (osMessageQueueGet(qCanRx, &frame, NULL, osWaitForever) == osOK) {
      int res = 0;
      // 1. 온습도 데이터(0x210) 처리
      if (frame.std_id == CAN_ID_SNS_ENV) {
        res = protocol_parse_env_v22(frame.data, frame.dlc, &sample.humidity, &sample.temperature);
      } 
      // 2. 조도 데이터(0x220) 처리
      else if (frame.std_id == CAN_ID_SNS_LUX) {
        res = protocol_parse_lux_v22(frame.data, frame.dlc, &sample.lux);
        if (res == 1) { // 성공적으로 해석됐다면?
          sample.tick_ms = osKernelGetTickCount(); // 받은 시간 기록 (타임아웃 감시용)
          osMessageQueuePut(qSensor, &sample, 0U, 0U); // 판단 태스크(Control)로 전송!
        }
      }

      // 무결성 검증 실패 시 로그 출력 (누가 나쁜 데이터를 쏘고 있는지 확인용)
      if (res < 0) {
        printf("!! Security Alert !! CRC/Sender Fail on [%03X]. Data discarded.\r\n", frame.std_id);
      }
    }
  }
}

/* 
 * [일꾼 B] ControlTask: 상황 판단 및 제어 명령 생성
 * 수집된 센서 데이터가 우리 기준(임계값)을 넘었는지 확인하고 명령을 내려.
 */
static void ControlTask(void *argument) {
    printf("Gateway Logic engine started (v2.2 Scenario)\r\n");
    static uint8_t token_cnt = 0; // 명령마다 고유번호(Token)를 붙여서 답장을 확인할 거야

    for (;;) {
        SensorSample_t sample;
        // 2초 동안 센서 데이터가 한 번도 안 오면? -> 센서 노드가 죽었다고 판단(Timeout)
        if (osMessageQueueGet(qSensor, &sample, NULL, 2000) == osOK) {
            
            // 시나리오 1: 조도 기반 전조등 제어
            ControlCommand_t light_cmd = {.target = TARGET_HEADLIGHT, .token = token_cnt++};
            // 1000 Lux 이하면 어둡다고 판단해서 라이트를 키도록 명령해
            light_cmd.action = (sample.lux < 1000) ? 1 : 0; 
            osMessageQueuePut(qControlCmd, &light_cmd, 0U, 0U);

            // 시나리오 2: 습도 기반 자동 와이퍼 제어 (v1.1 유지)
            ControlCommand_t wiper_cmd = {.target = TARGET_WIPER, .token = token_cnt++};
            if (sample.humidity >= 40) wiper_cmd.action = 2;      // HIGH
            else if (sample.humidity >= 30) wiper_cmd.action = 1; // LOW
            else wiper_cmd.action = 0;                            // OFF
            osMessageQueuePut(qControlCmd, &wiper_cmd, 0U, 0U);

            printf("[Log] H:%.1f%% T:%.1fC L:%d -> Lamp:%d Wipe:%d\r\n", 
                   sample.humidity, sample.temperature, sample.lux, light_cmd.action, wiper_cmd.action);
        } else {
            // [v2.2 안전 로직] 센서 값이 안 들어오면 시스템이 '장님'이 된 거야. 안전을 위해 ESTOP 선언!
            printf("!!! SYSTEM FAILSAFE !!! Sensor Timeout detected.\r\n");
            ControlCommand_t estop = {.target = 0, .action = 1, .token = 0xFF}; // 비상 상황 전파
            osMessageQueuePut(qControlCmd, &estop, 0U, 0U);
        }
    }
}

/* 
 * [일꾼 C] CanTxTask: 명령 송신 및 주기적 상태 보고
 * 액츄에이터에게 명령을 쏘고, 게이트웨이가 살아있음을 매초 보고해 (GW_STATE).
 */
static void CanTxTask(void *argument) {
  printf("Gateway CAN TX engine started\r\n");
  uint32_t last_gw_state_tick = 0;

  for (;;) {
    ControlCommand_t cmd;
    // 명령이 들어오면 지체 없이 쏴야 해. (100ms만 기다리고, 없으면 GW_STATE 주기 체크하러 감)
    if (osMessageQueueGet(qControlCmd, &cmd, NULL, 100) == osOK) {
        CAN_TxHeaderTypeDef txHeader = {.IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 4};
        uint8_t txData[4] = {0};

        if (cmd.target == 0 && cmd.token == 0xFF) { // [v2.2] 긴급 안전 전이 메시지 (ESTOP)
            txHeader.StdId = CAN_ID_GW_ESTOP;
            txHeader.DLC = 2; // 규격대로 DLC=2
            txData[0] = 1; // 원인 코드: 1=TIMEOUT
            txData[1] = 1; // 조치 코드: 1=SAFE_LIGHT_ON (라이트 켜고 정지)
        } else if (cmd.target == TARGET_HEADLIGHT) {
            txHeader.StdId = CAN_ID_CMD_LIGHT;
            txData[0] = 1; // 대상: HEAD
            txData[1] = cmd.action;
            txData[3] = cmd.token;
        } else if (cmd.target == TARGET_WIPER) {
            txHeader.StdId = CAN_ID_CMD_WIPER;
            txData[0] = cmd.action;
            txData[3] = cmd.token;
        }

        uint32_t mailbox;
        HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &mailbox);
    }

    // [v2.2 필수 사항] 1초마다 게이트웨이의 상태(GW_STATE)를 네트워크에 뿌려.
    // 그래야 액츄에이터 노드들이 "게이트웨이가 살아있구나"라고 안심하고 동작해.
    if (osKernelGetTickCount() - last_gw_state_tick >= 1000) {
        last_gw_state_tick = osKernelGetTickCount();
        CAN_TxHeaderTypeDef txHeader = {.StdId = CAN_ID_GW_STATE, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 4};
        uint8_t txData[4] = {0, 0, 0, 0}; // [0]: NORMAL 상태 의미
        uint32_t mailbox;
        HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &mailbox);
    }
  }
}

/* ISR: CAN 하드웨어에 데이터가 도착했을 때 OS 큐에 넣어주는 아주 빠른 함수 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef rxHeader;
  uint8_t rxData[8];
  if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK) {
    CanRawFrame_t frame = {.std_id = rxHeader.StdId, .dlc = rxHeader.DLC};
    memcpy(frame.data, rxData, 8);
    // OS 큐에 담아서 태스크들이 느긋하게 처리하게 해
    osMessageQueuePut(qCanRx, &frame, 0U, 0U);
  }
}
