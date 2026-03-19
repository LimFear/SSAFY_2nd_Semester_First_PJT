# Data Interface Specification (v2.1)
*(v2.0 기반 + Console-Gateway 로직 분리 및 규격 통일 반영)*

본 문서는 STM32(FreeRTOS) 게이트웨이가 주도하는 **Console-Gateway-Actuator** 3계층 시스템의 통신 규격을 정의한다. 모든 설계는 ISO 26262 기능 안전 철학을 반영한다.

---

## 1. 통신 계층 및 노드 식별 (Topology)

- **Baudrate**: 500 kbps / **DLC**: 8 (Fixed)
- **Node ID (Sender ID)**:
  - **1**: Gateway (STM32) - *Sole Commander*
  - **2**: Sensor Node (ESP32)
  - **3**: Actuator Node (ESP32)
  - **4**: Console Node (ESP32)

---

## 2. 메시지 분류 및 ID 명세

### 2.1 센서 수집 (SNS -> GW)
| CAN ID | 이름 | 설명 | Payload 정의 |
|:---:|:---:|---|---|
| **0x100** | **SNS_DHT** | 온습도 데이터 | B2-3: Humidity (x10, LE) / B4-5: Temp (x10, LE) |
| **0x110** | **SNS_LUX** | 조도 데이터 | B2-3: Lux Raw ADC (uint16, LE) |

### 2.2 모드 동기화 (CON -> GW) - **[NEW]**
| CAN ID | 이름 | 설명 | Payload 정의 |
|:---:|:---:|---|---|
| **0x300** | **CON_MODE_SET** | 사용자 설정 모드 공유 | B2: Wiper Mode / B3: Light Mode / B4: Turn Mode |
*Wiper(0:Auto, 1:Off, 2:Low, 3:High) / Light(0:Auto, 1:Off, 2:On) / Turn(0:Off, 1:Left, 2:Right, 3:Hazard)*

### 2.3 게이트웨이 명령 (GW -> ACT)
| CAN ID | 이름 | Payload (B2: Type / B3: Value) |
|:---:|:---:|---|
| **0x200** | **CMD_WIPER** | B2: 0x01 (Set Speed) / B3: Speed Level (0~3) |
| **0x210** | **CMD_LIGHT** | B2: 0x11 (Set State) / B3: State (0:OFF, 1:ON) |
| **0x220** | **CMD_TURN** | B2: 0x31 (Pulse) / B3: Dir(0:L, 1:R) <br> B2: 0x32 (Hazard) / B3: State(0/1) |

---

## 3. 제어 로직 및 안전 임계값

### 3.1 와이퍼 (Humidity 기반 AUTO)
- **FAST (3)**: 40.0% 이상
- **NORMAL (2)**: 35.0% 이상
- **SLOW (1)**: 30.0% 이상
- **OFF (0)**: 30.0% 미만

### 3.2 전조등 (Lux 기반 AUTO - Hysteresis 적용)
- **ON 진입**: 800 (ADC) 이상 (충분히 어두울 때)
- **OFF 진입**: 750 (ADC) 이하 (확실히 밝아질 때)
- **중간 영역**: 이전 상태를 유지하여 헌팅(깜빡임) 방지.

---

## 4. 안전 정책 (Safety Mechanisms)

1. **무결성 검증 (E2E-lite)**: 
   - 모든 수신 데이터는 `CRC8(SAE J1850)` 및 `Sender ID`가 일치할 때만 수용한다.
2. **명령 단일화**: 
   - 액추에이터는 오직 **Sender ID 1(Gateway)**이 보낸 명령만 수행한다.
3. **Failsafe (Fail-Operational)**:
   - 센서 데이터가 1.5초 이상 누락되거나 무결성 오류 3회 발생 시, 게이트웨이는 전 노드에 **0x080 (ESTOP)**을 브로드캐스트하고 와이퍼를 정지시킨다.
