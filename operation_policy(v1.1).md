# SSAFY 2학기 공통PJT

# CAN 기반 Body Control 시스템

## 인터페이스 & 운영 정책 v1.1

### 목적

- 센서(온습도/조도) → **STM32 Gateway 검증/판단** → 액추에이터(LED(전조,후미,깜빡)/와이퍼) 제어
- 모든 제어 권한과 안전 책임은 **STM32**에 있음
- 자율주차/Orin은 **별도 상위 기능**, 본 문서와 충돌하지 않음

---

## 1. 시스템 역할 정의

### Sensor ECU (ESP32)

- 온습도, 조도 센서 데이터 송신
- 판단/제어 로직 없음
- 주기 송신 + Alive 송신

### Gateway ECU (STM32)

- 모든 CAN 메시지 수신
- 데이터 검증(무결성/범위/타임아웃)
- 상태머신(NORMAL/WARNING/FAILSAFE) 관리
- **최종 제어 명령 생성 및 송신**

### Actuator ECU (ESP32)

- STM32 명령만 수신
- LED, 와이퍼(서보) 동작 수행
- 센서 데이터 직접 사용 ❌
- 명령 실행 결과 및 현재 상태를 ACT_FEEDBACK으로 보고

---

## 2. CAN ID & 메시지 규격 (확정)

### 우선순위 규칙

- **ID 숫자 작을수록 우선**
- 안전/명령 > 상태·피드백 > 센서

### CAN ID 맵

| ID(hex) | 이름 | 송신 주체 | 설명 |  |
| --- | --- | --- | --- | --- |
| 0x080 | GW_ESTOP | STM32 | 즉시 안전모드 | Gateway Emergency Stop |
| 0x110 | CMD_LIGHT | STM32 | LED 제어 | Command Light |
| 0x120 | CMD_WIPER | STM32 | 와이퍼 제어 | Command Wiper |
|  |  |  |  |  |
| 0x130 | ACT_FEEDBACK | Actuator ESP32 | 액추 실행 결과/상태 보고 | Actuator Feedback |
| 0x140 | GW_STATE | STM32 | Gateway 상태 | Gateway State |
|  |  |  |  |  |
| 0x210 | SNS_ENV | Sensor ESP32 | 온습도 | Sensor Environment |
| 0x220 | SNS_LUX | Sensor ESP32 | 조도 | Sensor Luminance |
| 0x2F0 | SNS_ALIVE | Sensor ESP32 | 센서 생존 | Sensor Alive |

---

## 3. Payload 요약 (최소 규격)

### GW_ESTOP (0x080, DLC=2)

STM32가 “지금부터 안전 모드다”를 모든 노드에 알리는 즉시 신호

- **B0: reason**
    - 0 = NONE(사용 안 함)
    - 1 = TIMEOUT (센서 타임아웃)
    - 2 = DATA_INVALID (범위/형식 오류)
    - 3 = INTEGRITY_FAIL (checksum/seq 연속 오류)
    - 4 = MANUAL_ESTOP (수동 E-STOP)
- **B1: action**
    - 0 = SAFE_DEFAULT (문서에 정의한 안전 상태로 전환)
    - 1 = SAFE_LIGHT_ON (조명만 강제 ON 같은 옵션; 필요 시)

### CMD_LIGHT (0x110, DLC=4)

- B0: target (HEAD/TAIL 등)
- B1: ON/OFF
- B2: BLINK
- B3: token
    - STM32가 부여하는 명령 식별자
    - ACT_FEEDBACK에서 token_echo로 반드시 회신되어야 함

### CMD_WIPER (0x120, DLC=4)

- B0: mode (OFF/LOW/HIGH)
- B1: level
- B3: token
    - STM32가 부여하는 명령 식별자
    - ACT_FEEDBACK에서 token_echo로 반드시 회신되어야 함

### ACT_FEEDBACK (0x130, DLC=4)

- B0: target
    - 1 = HEAD_LIGHT
    - 2 = TAIL_LIGHT
    - 3 = WIPER
- B1: result
    - 0 = OK (정상 실행)
    - 1 = FAIL (실행 실패)
    - 2 = INVALID (명령 값 이상)
    - 3 = TIMEOUT (액추 내부 타임아웃)
- B2: current_state
    - Light: 0=OFF, 1=ON, 2=BLINK
    - Wiper: 0=OFF, 1=LOW, 2=HIGH
- B3: token_echo
    - STM32가 송신한 CMD의 token을 그대로 회신

### GW_STATE (0x140, DLC=4)

STM32가 현재 시스템 상태를 주기적으로 브로드캐스트 (모니터링/디버깅용)

- **B0: state**
    - 0 = NORMAL
    - 1 = WARNING
    - 2 = FAILSAFE
- **B1: last_err**
    - 0 = NONE
    - 1 = TIMEOUT
    - 2 = CHECKSUM
    - 3 = SEQ
    - 4 = RANGE
    - 5 = ACT_FAIL
- **B2: err_count**
    - 오류 누적 카운트(0~255)
- **B3: reserved**
    - 0

### SNS_ENV (0x210, DLC=8)

- B2~3: 온도 ×100 (int16)
- B4: 습도(0~100)
- B5: seq
- B7: checksum

### SNS_LUX (0x220, DLC=8)

- B2~3: 조도(lux)
- B5: seq
- B7: checksum

### SNS_ALIVE (0x2F0, DLC=2)

“이 ESP32 Sensor ECU가 살아있다” 신호 (센서 값 X, 노드 생존 O)

- **B0: ecu_id**
    - 1 = Sensor ECU
- **B1: alive_counter**
    - 0~255 순환 증가

---

## 4. 운영 정책 (핵심)

### 상태 머신

- **NORMAL**: 정상 제어
- **WARNING**: 데이터 이상 감지, 동작 유지
- **FAILSAFE**: 안전 모드, 제한 동작

### FAILSAFE 진입 조건

- 센서 타임아웃(>1.5초)
- 무결성/범위 오류 연속 3회
- 명시적 E-STOP 수신
- 액추 실행 실패(result=FAIL) 연속 발생
- 액추 피드백 미수신(명령 확인 불가)

### FAILSAFE 동작

- LED: **안전 상태** (팀 합의값)
- 와이퍼: OFF
- 상태 브로드캐스트(GW_STATE)

### 자동 복구

- FAILSAFE → 즉시 NORMAL ❌
- 안정 시간 + 정상 프레임 연속 수신 시 WARNING로만 복귀 가능

---

## 5. 권한 우선순위

1. **E-STOP / FAILSAFE**
2. 수동 입력(키보드/조이스틱)
3. 자율주차/AI(Orin)

> 어떤 경우에도 액추에이터는 STM32 명령만 따른다.
> 

---

## 6. 통합 테스트 시나리오 V1.1(10개)

### 공통 전제(고정값)

- 센서 주기: 500ms
- 타임아웃: 1500ms(=3주기)
- FAILSAFE 조건: 연속 오류 3회 또는 타임아웃
- 상태: NORMAL / WARNING / FAILSAFE
- 명령 우선순위: E-STOP(0x080) > CMD(0x110/0x120) > SNS(0x210/0x220) > STATE(0x140)

---

### 시나리오 표

### 1) 조도 낮음 → 전조등 ON

- **입력/조건**: LUX < LUX_LOW_TH
- **기대 동작**: STM32가 `CMD_LIGHT(HEAD, ON)` 송신, 액추 LED ON
- **합격 기준**: 1초 이내 LED ON, 상태 NORMAL 유지
- **로그 포인트**: `SNS_LUX decode`, `CMD_LIGHT TX`

---

### 2) 조도 높음 → 전조등 OFF

- **입력/조건**: LUX > LUX_HIGH_TH
- **기대 동작**: `CMD_LIGHT(HEAD, OFF)` 송신, LED OFF
- **합격 기준**: 1초 이내 LED OFF, NORMAL 유지
- **로그 포인트**: 히스테리시스 적용 여부(불필요한 ON/OFF 반복 방지)

---

### 3) 습도 높음(또는 온습도 조건) → 와이퍼 ON

- **입력/조건**: HUM ≥ HUM_TH
- **기대 동작**: `CMD_WIPER(LOW 또는 HIGH)` 송신, 서보 구동
- **합격 기준**: 1초 이내 와이퍼 동작, NORMAL 유지
- **로그 포인트**: `SNS_ENV decode`, `CMD_WIPER TX`

---

### 4) 센서 ECU 전원 OFF → 타임아웃 FAILSAFE

- **입력/조건**: 센서 ESP32 전원 차단
- **기대 동작**: 1500ms 내 `FAILSAFE` 진입, `GW_ESTOP` + 안전 명령 송신
- **합격 기준**: 1.5초±α 내 FAILSAFE, LED/와이퍼 안전 상태로 전환
- **로그 포인트**: `timeout detect`, `GW_ESTOP TX`, `GW_STATE=FAILSAFE`

---

### 5) 단발 checksum 오류 1회 → WARNING (완충)

- **입력/조건**: 센서가 checksum 일부러 1회 틀리게 송신
- **기대 동작**: 해당 프레임 DROP, `WARNING` 전이(또는 warn_streak 증가)
- **합격 기준**: 시스템이 즉시 FAILSAFE로 가지 않고 WARNING 처리
- **로그 포인트**: `CSUM fail`, `drop`, `warn_streak`

---

### 6) checksum 오류 3회 연속 → FAILSAFE

- **입력/조건**: checksum 오류 연속 3회 송신
- **기대 동작**: `FAILSAFE` 진입, `GW_ESTOP` 송신, 안전 명령 실행
- **합격 기준**: 3회 이내 FAILSAFE 확정, 액추 안전상태
- **로그 포인트**: `warn_streak>=3`, `enter_failsafe(reason=CSUM)`

---

### 7) seq 이상(점프/중복) → WARNING, 연속 시 FAILSAFE

- **입력/조건**: seq를 고정하거나 10씩 점프
- **기대 동작**: WARNING 처리(바로 FS 금지), 연속 반복 시 FS
- **합격 기준**: 1~2회는 WARNING, 3회 이상 반복되면 FAILSAFE(정책에 따라)
- **로그 포인트**: `SEQ mismatch`, `warn_streak`

---

### 8) 명령–상태 확인 실패 (ACT_FEEDBACK 미수신)

- **입력/조건**: 액추 ECU에서 피드백 송신 기능 OFF(또는 CAN선 분리)
- **기대 동작**: STM32가 “명령 미확인”으로 WARNING(또는 재송신 N회)
- **합격 기준**: 피드백 타임아웃 로그 발생 + 상태 전이(WARNING 이상)
- **로그 포인트**: `pending_token timeout`, `retry_count`, `GW_STATE`

> 옵션: 재송신 N회 후 FAILSAFE로 전환하면 더 설득력 좋음.
> 

---

### 9) 액추 실행 실패 피드백(result=FAIL)

- **입력/조건**: 액추가 의도적으로 result=FAIL 보내기(서보 분리 등)
- **기대 동작**: WARNING 전이 + 안전 명령(와이퍼 OFF 등)
- **합격 기준**: "명령은 보냈으나 상태 달성 실패" 감지 ⭕
- **로그 포인트**: `ACT_FAIL`, `send_safe_cmds`

---

### 10) 부분 센서 고장(조도만/온습도만 단선)

- **입력/조건 A**: `SNS_LUX`만 타임아웃, `SNS_ENV`는 정상
- **기대 동작 A**: 조명은 안전 상태(예: HEAD ON), 와이퍼는 정상 로직 유지
- **합격 기준 A**: 전체 FAILSAFE로 과도하게 내려가지 않거나, 도메인별 degraded 처리
- **입력/조건 B**: `SNS_ENV`만 타임아웃, `SNS_LUX`는 정상
- **기대 동작 B**: 와이퍼 안전 상태(OFF), 조명은 정상 로직 유지
- **합격 기준 B**: 부분 고장에서도 나머지 기능 유지
- **로그 포인트**: `timeout per-signal`, `domain_state`(가능하면)

> 프로젝트 단순화를 위해 “전체 FAILSAFE”로 가도 되지만,
포트폴리오 완성도를 올리려면 부분 고장은 부분 degraded가 더 좋음.
> 

---

## 추천: 데모 당일 “필수 5개”만 찍고 나머지는 로그로 증명

- 필수(영상/실물 데모): 1,3,4,6,10
- 나머지(로그/캡처): 5,7,8,9

---

### 합의 사항

- 본 문서는 **v1.1 기준선**
- 자율주차/Orin 기능은 **본 규격을 침범하지 않음**

---