# CAN 기반 Body Control System
## Interface & Operation Policy v2.2
*(ISO 26262 + ISO 11898 기반 안정성 설계 / Payload Complete Edition)*

---

## 0. 문서 목적 및 버전 이력

본 문서는 CAN 기반 Body Control 시스템에서  
통신 신뢰성(ISO 11898)과 기능 안전(ISO 26262)을 기준으로  
ECU 역할, CAN ID, Payload, 고장 대응 정책을 정의한다.

### 버전 이력
- v1.1: 기본 인터페이스 및 운영 정책
- v2.0: ISO 26262 + ISO 11898 설계 반영
- v2.1: 리뷰 대응 보강 (책임 경계 / 확장성 명문화)
- **v2.2: 모든 메시지 Payload 명세 추가 (계약서 완성)**

---

## 1. 시스템 아키텍처 및 책임

### Sensor ECU (ESP32)
- 온습도 / 조도 센서 데이터 측정
- 판단 로직 없음
- 주기 송신 + Alive 송신
- E2E-lite 무결성 보호 수행

### Gateway ECU (STM32)
- 시스템 유일 판단 주체
- 모든 CAN 메시지 검증 (무결성 / 범위 / 타임아웃)
- 상태머신 관리 (NORMAL / WARNING / FAILSAFE)
- 최종 제어 명령 및 안전 전이 송신

### Actuator ECU (ESP32)
- Gateway 명령만 수신
- 실행 결과 및 상태를 피드백
- Gateway 침묵 시 독립 FAILSAFE 수행

---

## 2. Safety Goals (ISO 26262)

- **SG-1**: 센서 오류 시 액추에이터 오동작 방지
- **SG-2**: Gateway 침묵 시 액추에이터 독립 안전 전이
- **SG-3**: 명령 실행 실패 지속 시 제어 중단 및 안전 유지

---

## 3. CAN ID 우선순위 (ISO 11898)

| ID(hex) | 이름 | 송신 ECU | 설명 |
|---|---|---|---|
| 0x080 | GW_ESTOP | Gateway | 긴급 안전 전이 |
| 0x110 | CMD_LIGHT | Gateway | 조명 제어 |
| 0x120 | CMD_WIPER | Gateway | 와이퍼 제어 |
| 0x130 | ACT_FEEDBACK | Actuator | 실행 결과 |
| 0x140 | GW_STATE | Gateway | Gateway 상태 |
| 0x210 | SNS_ENV | Sensor | 온습도 |
| 0x220 | SNS_LUX | Sensor | 조도 |
| 0x2F0 | SNS_ALIVE | Sensor | 센서 생존 |

---

## 4. E2E-lite Payload 공통 규칙

모든 **안전 관련 메시지**는 아래 공통 구조를 따른다.

| Byte | 필드 | 설명 |
|---|---|---|
| B0 | sender_id | ECU 식별자 |
| B1 | rolling_counter | 순서/중복 검출 |
| B2~B6 | payload | 메시지별 데이터 |
| B7 | crc8 | 논리 무결성 |

※ CAN 하드웨어 CRC와 별도 (ISO 26262 진단 목적)

---

## 5. 메시지별 Payload 정의 (완전 명세)

---

### 5.1 GW_ESTOP (0x080, DLC=2)

Gateway가 즉시 안전 전이를 선언하는 메시지

| Byte | 필드 | 설명 |
|---|---|---|
| B0 | reason_code | 1=TIMEOUT, 2=DATA_INVALID, 3=INTEGRITY_FAIL, 4=ACT_FAIL, 5=MANUAL |
| B1 | action_code | 0=SAFE_DEFAULT, 1=SAFE_LIGHT_ON |

---

### 5.2 CMD_LIGHT (0x110, DLC=4)

| Byte | 필드 | 설명 |
|---|---|---|
| B0 | target | 1=HEAD, 2=TAIL |
| B1 | on_off | 0=OFF, 1=ON |
| B2 | blink | 0=NO, 1=YES |
| B3 | token | 명령 식별자 |

---

### 5.3 CMD_WIPER (0x120, DLC=4)

| Byte | 필드 | 설명 |
|---|---|---|
| B0 | mode | 0=OFF, 1=LOW, 2=HIGH |
| B1 | level | 세기/속도 |
| B2 | reserved | 0 |
| B3 | token | 명령 식별자 |

---

### 5.4 ACT_FEEDBACK (0x130, DLC=4)

| Byte | 필드 | 설명 |
|---|---|---|
| B0 | target | 1=HEAD, 2=TAIL, 3=WIPER |
| B1 | result | 0=OK, 1=FAIL, 2=INVALID, 3=TIMEOUT |
| B2 | current_state | Light: OFF/ON/BLINK, Wiper: OFF/LOW/HIGH |
| B3 | token_echo | CMD의 token 그대로 회신 |

---

### 5.5 GW_STATE (0x140, DLC=4)

Gateway 상태 브로드캐스트 (모니터링 + 생존 판단 보조)

| Byte | 필드 | 설명 |
|---|---|---|
| B0 | state | 0=NORMAL, 1=WARNING, 2=FAILSAFE |
| B1 | last_error | 0=NONE, 1=TIMEOUT, 2=CRC, 3=SEQ, 4=RANGE, 5=ACT_FAIL |
| B2 | error_count | 누적 오류 카운트 |
| B3 | reserved | 0 |

---

### 5.6 SNS_ENV (0x210, DLC=8)

| Byte | 필드 | 설명 |
|---|---|---|
| B2-3 | temperature | °C ×100 (int16) |
| B4 | humidity | 0~100 % |
| B5 | seq | rolling counter |
| B7 | checksum | crc8 |

---

### 5.7 SNS_LUX (0x220, DLC=8)

| Byte | 필드 | 설명 |
|---|---|---|
| B2-3 | lux | 조도 값 |
| B5 | seq | rolling counter |
| B7 | checksum | crc8 |

---

### 5.8 SNS_ALIVE (0x2F0, DLC=2)

| Byte | 필드 | 설명 |
|---|---|---|
| B0 | ecu_id | Sensor ECU ID |
| B1 | alive_counter | 생존 카운터 |

---

## 6. 상태머신 및 FAILSAFE 정책

- **NORMAL**: 정상 제어
- **WARNING**: 일시 오류, 제어 유지
- **FAILSAFE**: 안전 상태, 기능 제한

### FAILSAFE 진입 조건
- 센서 타임아웃 (>1500ms)
- 무결성/범위 오류 연속 3회
- GW_ESTOP 수신
- ACT_FEEDBACK 실패 연속
- CAN Bus-off

---

## 7. Actuator 독립 FAILSAFE 규칙

- CMD_* 또는 GW_STATE 미수신 T_dead 초 초과 시:
  - 모든 동작 중단
  - 사전 정의된 안전 상태 유지

---

## 8. 확장성 가이드

- 센서/액추 ECU 추가 시 Payload 구조 재사용
- 상위 제어기(Jetson/Orin) 연계 가능
- AUTOSAR Classic 개념과 구조 충돌 없음

---

## 9. ISO 기준 반영 근거 정리

### ISO 11898
- Arbitration 기반 ID 우선순위
- Error frame / Bus-off 처리
- Deterministic CAN traffic

### ISO 26262
- Safety Goal 정의
- Timeout / Counter / CRC 진단
- FAILSAFE 상태머신
- Single Point of Failure 대응 (Actuator 독립 안전)

---

## 10. 설계 철학 요약

- 통신은 ISO 11898에 위임
- 안전 판단은 ISO 26262 관점에서 Gateway가 수행
- 단순하지만 설명 가능한 구조
- 운영·관제·FAE에 적합한 전장 설계
