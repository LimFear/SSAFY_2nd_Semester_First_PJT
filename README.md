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


