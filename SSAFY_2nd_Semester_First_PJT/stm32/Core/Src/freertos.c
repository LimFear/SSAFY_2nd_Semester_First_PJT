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
static osMessageQueueId_t qSysMode;    // 콘솔로부터 받은 시스템 운영 모드 (v2.0)

/* OS Tasks: 동시에 일하는 3명의 일꾼이야 */
static osThreadId_t canRxTaskHandle;
static osThreadId_t controlTaskHandle;
static osThreadId_t canTxTaskHandle;

/* MX_FREERTOS_Init: 운영 인터페이스 초기화 (큐와 태크스를 만들어) */
void MX_FREERTOS_Init(void) {
  qCanRx      = osMessageQueueNew(32, sizeof(CanRawFrame_t), NULL);
  qSensor     = osMessageQueueNew(8,  sizeof(SensorSample_t), NULL);
  qControlCmd = osMessageQueueNew(8,  sizeof(ControlCommand_t), NULL);
  qSysMode    = osMessageQueueNew(4,  sizeof(SystemMode_t), NULL);

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
      // 1. 온습도 데이터(0x100) 처리
      if (frame.std_id == CAN_ID_SNS_DHT) {
        res = protocol_parse_env_v22(frame.data, frame.dlc, &sample.humidity, &sample.temperature);
      } 
      // 2. 조도 데이터(0x110) 처리
        else if (frame.std_id == CAN_ID_SNS_LUX) {
          res = protocol_parse_lux_v22(frame.data, frame.dlc, &sample.lux);
          if (res == 1) {
            sample.tick_ms = osKernelGetTickCount();
            osMessageQueuePut(qSensor, &sample, 0U, 0U);
          }
        }
        else if (frame.std_id == CAN_ID_CON_MODE_SET) {
          // E2E 검증: CRC(B7) + Sender ID(B0) 검사 (Console ID는 4)
          if (frame.data[7] == protocol_calculate_crc8(frame.data, 7) && frame.data[0] == 4) {
            SystemMode_t mode = {
                .wiper_mode = frame.data[2], 
                .light_mode = frame.data[3],
                .turn_mode  = frame.data[4]
            };
            osMessageQueuePut(qSysMode, &mode, 0U, 0U);
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
    printf("Gateway Logic engine started (v2.0 Console-Sync)\r\n");
    static uint8_t token_cnt = 0;
    SystemMode_t current_mode = {0, 0}; // Default: Both AUTO
    static uint8_t last_light_action = 0; 

    for (;;) {
        // 1. 콘솔로부터 새로운 모드 설정이 왔는지 확인 (Non-blocking)
        SystemMode_t new_mode;
        if (osMessageQueueGet(qSysMode, &new_mode, NULL, 0) == osOK) {
            current_mode = new_mode;
            printf("[Mode] Synced from Console - Wiper:%d Light:%d\r\n", current_mode.wiper_mode, current_mode.light_mode);
        }

        SensorSample_t sample;
        if (osMessageQueueGet(qSensor, &sample, NULL, 2000) == osOK) {
            
            // 시나리오 1: 전조등 제어 (800 ON / 750 OFF 히스테리시스 적용)
            ControlCommand_t light_cmd = {.target = TARGET_HEADLIGHT, .token = token_cnt++};
            if (current_mode.light_mode == 0) { // AUTO
                if (sample.lux >= 800) light_cmd.action = 1;      // 어두움 -> ON
                else if (sample.lux <= 750) light_cmd.action = 0; // 밝음 -> OFF
                else light_cmd.action = last_light_action;        // 유지 (Hunting 방지)
            } else { // Manual ON(2) or OFF(1)
                light_cmd.action = (current_mode.light_mode == 2) ? 1 : 0;
            }
            last_light_action = light_cmd.action;
            osMessageQueuePut(qControlCmd, &light_cmd, 0U, 0U);

            // 시나리오 2: 와이퍼 제어 (30/35/40 임계값 3단계 반영)
            ControlCommand_t wiper_cmd = {.target = TARGET_WIPER, .token = token_cnt++};
            if (current_mode.wiper_mode == 0) { // AUTO
                if (sample.humidity >= 40.0f)      wiper_cmd.action = 3; // FAST
                else if (sample.humidity >= 35.0f) wiper_cmd.action = 2; // NORMAL
                else if (sample.humidity >= 30.0f) wiper_cmd.action = 1; // SLOW
                else                               wiper_cmd.action = 0; // OFF
            } else { // Manual OFF(1), LOW(2), HIGH(3)
                wiper_cmd.action = (current_mode.wiper_mode >= 1) ? (current_mode.wiper_mode - 1) : 0;
            }
            osMessageQueuePut(qControlCmd, &wiper_cmd, 0U, 0U);

            // 시나리오 3: 방향지시등 제어 (심리스 상태 전이)
            static uint8_t last_turn = 0; // 0:OFF, 1:L, 2:R, 3:H
            if (current_mode.turn_mode != last_turn) {
                ControlCommand_t turn_cmd = {.target = TARGET_TURN, .token = token_cnt++};
                
                // [규칙] OFF로 갈 때는 "어디서 왔는가"가 중요함 (Toggle 방식 때문)
                if (current_mode.turn_mode == 0) { // To OFF
                    if (last_turn == 1) turn_cmd.action = 0x11; // OFF from LEFT
                    else if (last_turn == 2) turn_cmd.action = 0x12; // OFF from RIGHT
                    else turn_cmd.action = 0; // Just general OFF (Hazard)
                } else {
                    turn_cmd.action = current_mode.turn_mode;
                }
                
                last_turn = current_mode.turn_mode;
                osMessageQueuePut(qControlCmd, &turn_cmd, 0U, 0U);
            }

            printf("[Log] L:%d H:%.1f -> Lamp:%d Wipe:%d (Modes:%d/%d)\r\n", 
                   sample.lux, sample.humidity, light_cmd.action, wiper_cmd.action, 
                   current_mode.light_mode, current_mode.wiper_mode);
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
  printf("Gateway CAN TX engine started (v2.0 Safe Frame)\r\n");
  uint32_t last_gw_state_tick = 0;
  static uint8_t rolling_cnt = 0;

  for (;;) {
    ControlCommand_t cmd;
    if (osMessageQueueGet(qControlCmd, &cmd, NULL, 100) == osOK) {
        CAN_TxHeaderTypeDef txHeader = {.IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8};
        uint8_t txData[8] = {0};

        // B0: Sender ID (1), B1: Rolling Counter
        txData[0] = 1; 
        txData[1] = rolling_cnt++;

        if (cmd.target == 0 && cmd.token == 0xFF) { // ESTOP
            txHeader.StdId = CAN_ID_GW_ESTOP;
            txData[2] = 1; // Reason: TIMEOUT
            txData[3] = 1; // Action: SAFE_LIGHT_ON
        } else if (cmd.target == TARGET_HEADLIGHT) {
            txHeader.StdId = CAN_ID_CMD_LIGHT;
            txData[2] = 0x11; // Type: Set State
            txData[3] = cmd.action;
            txData[6] = cmd.token;
        } else if (cmd.target == TARGET_WIPER) {
            txHeader.StdId = CAN_ID_CMD_WIPER;
            txData[2] = 0x01; // Type: Set Speed Level
            txData[3] = cmd.action;
            txData[6] = cmd.token;
        } else if (cmd.target == TARGET_TURN) {
            txHeader.StdId = CAN_ID_CMD_TURN;
            if (cmd.action == 3) { // HAZARD ON
                txData[2] = 0x32; // TURN_CMD_HAZARD_SET
                txData[3] = 1;    // Enable
            } else if (cmd.action == 1 || cmd.action == 0x11) { // LEFT Pulse
                txData[2] = 0x31; // TURN_CMD_PULSE
                txData[3] = 0x00; // DIR_LEFT
            } else if (cmd.action == 2 || cmd.action == 0x12) { // RIGHT Pulse
                txData[2] = 0x31; // TURN_CMD_PULSE
                txData[3] = 0x01; // DIR_RIGHT
            } else if (cmd.action == 0) { // Hazard OFF
                txData[2] = 0x32; // TURN_CMD_HAZARD_SET
                txData[3] = 0;    // Disable
            }
            txData[6] = cmd.token;
        }

        // B7: CRC8
        txData[7] = protocol_calculate_crc8(txData, 7);

        uint32_t mailbox;
        HAL_CAN_AddTxMessage(&hcan1, &txHeader, txData, &mailbox);
    }

    if (osKernelGetTickCount() - last_gw_state_tick >= 1000) {
        last_gw_state_tick = osKernelGetTickCount();
        CAN_TxHeaderTypeDef txHeader = {.StdId = CAN_ID_GW_STATE, .IDE = CAN_ID_STD, .RTR = CAN_RTR_DATA, .DLC = 8};
        uint8_t txData[8] = {0};

        txData[0] = 1; // Sender ID
        txData[1] = rolling_cnt++;
        txData[2] = 0; // Mode: NORMAL
        txData[7] = protocol_calculate_crc8(txData, 7);

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
