# SSAFY_2nd_Semester_First_PJT
SSAFY 14기 공통 프로젝트

### git 버전 관리 원칙
1. 공통 개발 branch는 master 혹은 main 으로 하며, 개인별로 개발하는 환경은 로컬 branch로 한다.
2. main 혹은 master branch 로 commit 하지 않고 로컬 branch로 commit 후 pull request 승인 받은 다음, merge 처리를 최우선 원칙으로 한다.
3. commit 하기 전에 LOG 작성 후 진행한다.

---
### Git LOG 작성 규칙
1. 작성 폼은 다음과 같다.

- (연도/월/일) 로그_순서. 이름[작업 결과]
- 작업_내용

ex) 

(26/01/12) 01. 홍길동[구현 완료] 
- README 작성

(26/01/12) 02. 김철수[오류 발생] 
- 구동부 후진 작동 이상 발생함

(26/01/13) 01. 김철수[오류 해결] 
- **(26/01/12) 02** 의 구동부 후진 작동 오류 해결함.



2. 작업 결과에는 구현 완료, 오류 발생, 오류 해결 등등 작성 가능하다.
3. 작업 내용에는 직관적으로 작성한다.
4. commit 하기 전에 LOG 작성 후 진행한다.
5. 날짜가 지나고 나서 첫 push 일 때, 로그 순서는 1번부터 시작하며, push마다 차례로 올라간다.
---
### GIT LOG
(26/01/12) 01. 임정환[구현 완료] 
- README 작성
- GIT 버전 관리 규칙 설정
- LOG 작성 규칙 설정

(26/01/15) 01. 임정환[구현 완료] 
- stm32(STM32F429I-DISC1) 환경 세팅 완료
    1. 프로그램
        -> IDE : STM32 CubeIDE, v1.9.0 (https://www.st.com/en/development-tools/stm32cubeide.html)
        -> tool : STM32 CubeMX, v6.16.1 (https://www.st.com/en/development-tools/stm32cubemx.html)

    2. 출력 테스트 확인을 위한 .ioc 세팅
        -> usart1 ==> Asynchronous 로 설정
        -> Project Manager -> Code Generator -> Generate files -> Generate Peripheral ... 체크

- 출력 테스트를 위한 코드 작성 후 정상 작동 확인함.

(26/01/19) 01. 임정환[구현 완료] 
- ESP32 개발 환경 설정
    1. Arduino IDE v.2.3.7 설치
    2. ESP32 DEV Module 설정 (필요시 드라이버 다운)

- DHT11 테스트 완료
    1. DATA 핀은 ESP32 GPIO16 번에 연결
    2. V_DD = 3.3V
    3. 라이브러리 "DHT sensor library" by Adafruit 를 설치함.

(26/01/20) 01. 임정환[구현 완료] 
- ESP32 끼리 CAN통신 연결 성공
1. 하나는 DHT11 연결해서 송신하고, 나머지는 수신함
2. RX => GPIO32, TX => GPIO33 으로 연결

(26/01/21) 01. 임정환[구현 완료] 
- STM32 <=> ESP32 CAN통신 연결 성공
1. Polling 방식이며, Inturrupt 방식으로 변환 필요함
2. ESP32 코드는 그대로 유지함
3. STM32 .ioc 설정은 다음과 같다.
    - Connenctivity CAN1을 Activated
    - 그 다음, Parameter Settings에서 다음과 같다. (Baud Rate => 500,000 bit/s 목적)
        - Prescaler = 2
        - Time Quanta in Bit Segment 1(줄여서 a.k.a TQ) = 11
        - TQ2 = 4

    - PA11 = Rx, PA12 = Tx

(26/01/22) 01. 임정환[구현 완료] 
1. ESP32 에서 control.c로 서보모터 구현함.
    - 습도를 기준으로 돌아가며, 30, 35, 40도 기준으로 습도가 올라갈 수록 모터 속도도 빨라짐.
    - STM32에서 각도 대신 Level 값으로 받으며, 0일 때는 30도 미만일 때로, 정지 
    - 서보 모터는 5V, GPIO 17번으로 지정했다.

2. STM32의 동작 구조를 FreeRTOS로 구현함.
    - CMSIS_V2로 실행함
        1. 우선 순위를 "1. 레벨 값 전송, 2. 데이터 처리, 3. 송신 데이터 인터럽트 처리" 로 정했다.

    - 센서 데이터를 인터럽트 방식으로 받으며, 이에 따라 제어를 판단하여 control에다 신호를 전송한다. 서보 모터는 레벨 데이터를 전송하여 30도일때 Level1, 35 = level2, 이런 식으로 순차적으로 전송한다.

3. STM32의 전체 코드를 push함

(26/01/23) 01. 임정환[구현 완료] 
1. MQTT 통신 설정 완료
- 개인 핫스팟으로 태블릿 <=> ESP32 보드 연결 성공함
- wifi ID, PW 환경마다 수정 필요
- LED를 ESP32 GPIO 27번에다 연결하였으며, ON, OFF 메시지로 제어함.
    1. ON -> LED가 켜짐
    2. OFF -> LED가 꺼짐

2. 실행 방법
- ESP32 보드에다 업로드 성공하면, 시리얼 모니터에 WiFi connented 옆에 ip가 뜸.
(ex. WiFi connected. 10.95.150.27)
- URL을 인터넷에 "http://ip번호" 작성 후 접속. 그럼 mqtt.c 파일 안에 있는 html이 열림.
- 인터넷 연결 확인 후, ON, OFF 버튼 눌러서 확인할 것.

(26/01/27) 01. 김영진[구현 완료]

0. 구현 개요
    - CDS 광조도센서 → ESP 센서부 → STM32 게이트웨이 → ESP 동작부 → LED(내부)  
      전체 CAN 기반 제어 흐름 구현 완료
    - 개발 환경
      - STM32CubeIDE v1.19
      - ESP-IDF
    - 예정 사항
      - 외부 LED 모듈로 확장 예정
      - 기존 온습도(DHT11) 제어 로직과 핀 번호 / 로직 충돌 예상
        - 추후 merge 시 통합 예정

1. ESP 센서부
핀 번호 (Pin Assignment)

| 기능 | 핀 번호 (GPIO) | 설명 |
| :--- | :---: | :--- |
| CAN TX | 33 | TWAI(CAN) 데이터 송신 |
| CAN RX | 32 | TWAI(CAN) 데이터 수신 |
| ADC Sensor | 34 | 조도 센서 (ADC1_CH6) 입력 |

데이터 패킷 설명 (CAN Data Protocol)
- ID: `0x101`
- DLC: `8` (실제 데이터는 Byte 0~1 사용)
  
| Byte | 역할 | 값 범위 | 설명 |
| :---: | :--- | :---: | :--- |
| Byte 0 | ADC High | 0 ~ 15 | ADC Raw 상위 8비트 |
| Byte 1 | ADC Low | 0 ~ 255 | ADC Raw 하위 8비트 |
| Byte 2~7 | Reserved | 0 | 미사용 |

동작 설명
- 조도 센서를 200ms 주기로 ADC(12-bit) 계측
- 측정 데이터를 CAN 메시지로 송신
- CAN 통신 속도: 500 kbps (TWAI, Normal Mode)

2. STM32 게이트웨이
핀 번호 (Pin Assignment)

| 기능 | Port / Pin | 설명 |
| :--- | :---: | :--- |
| CAN1 TX | PA12 | CAN 데이터 송신 |
| CAN1 RX | PA11 | CAN 데이터 수신 |
| UART1 TX | PA9 | 디버그 로그 출력 |
| UART1 RX | PA10 | 디버그 명령 수신 |

데이터 패킷 설명 (CAN Data Protocol)
수신 (From ESP Sensor)
- ID: `0x101`
- `rxData[0]`(High) + `rxData[1]`(Low) → 16-bit 조도값 복원
송신 (To ESP Actuator)
- ID: `0x201`
- DLC: `8`

| Byte | 역할 | 값 | 설명 |
| :---: | :--- | :---: | :--- |
| Byte 0 | LED CMD | 0 / 1 | 0: OFF, 1: ON |
| Byte 1~7 | Padding | 0 | Reserved |

동작 설명
- CAN 인터럽트 기반 메시지 수신
- 조도값 기준 임계값(1000) 비교
  - 초과 → LED ON
  - 이하 → LED OFF
- 제어 명령을 CAN 메시지(`0x201`)로 송신
- UART 로그로 상태 출력

3. ESP 동작부
핀 번호 (Pin Assignment)

| 기능 | GPIO | 설명 |
| :--- | :---: | :--- |
| CAN TX | 33 | TWAI(CAN) 송신 |
| CAN RX | 32 | TWAI(CAN) 수신 |
| LED RED | 4 | 내부 LED (임시) |
| LED GREEN | 5 | RGB LED Green |
| LED BLUE | 18 | RGB LED Blue |

데이터 패킷 설명 (CAN Data Protocol)
- ID: `0x201`
- DLC: `3` (또는 8 사용 시 Byte 0~2만 유효)

| Byte | 역할 | 값 | 설명 |
| :---: | :--- | :---: | :--- |
| Byte 0 | Red | 0 / 1 | OFF / ON |
| Byte 1 | Green | 0 / 1 | OFF / ON |
| Byte 2 | Blue | 0 / 1 | OFF / ON |

동작 설명
- CAN 메시지 수신 대기
- ID `0x201` 메시지만 처리
- 수신 데이터에 따라 RGB LED 제어
- 동작 상태 로그 출력

**(26/01/27) 02. 김영진[구현 완료]**
 - [CDS + DHT11 + 운영정책] 통합본 업로드 > 자세한 readme는 READ_ME_for_integration참고
