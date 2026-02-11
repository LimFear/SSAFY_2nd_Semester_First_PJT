#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <math.h>

#include "driver/twai.h"

/* =========================
 * WiFi / MQTT 설정
 * ========================= */
static const char* WIFI_SSID = "YOUR_ID";
static const char* WIFI_PASS = "YOUR_PASSWORD";

static const char* MQTT_HOST = "broker.emqx.io";
static const uint16_t MQTT_PORT = 1883;

/* 토픽 베이스 */
static const char* TOPIC_BASE = "YOUR_TOPIC";

/* =========================
 * 옵션
 * ========================= */
#define ENABLE_MQTT_COMMANDS 0  /* 1: MQTT 토픽으로 ON/OFF/AUTO 제어 허용, 0: 웹(UI)만 사용 */


/* =========================
 * CAN 핀/ID
 * ========================= */
#define CAN_RX_GPIO_PIN GPIO_NUM_32
#define CAN_TX_GPIO_PIN GPIO_NUM_33

#define CAN_DHT_ID              0x100
#define CAN_CDS_ID              0x110

#define CAN_ID_CMD_SERVO        0x200
#define SERVO_CMD_SET_SPEED_LEVEL   0x01
/* 호환용(기존 명칭) */
#define SERVO_CMD_SET_ANGLE           SERVO_CMD_SET_SPEED_LEVEL

#define CAN_ID_CMD_HIGH_BEAM    0x210
#define HIGH_BEAM_CMD_SET_STATE 0x11


#define CAN_ID_CMD_TURN           0x220
#define TURN_CMD_PULSE            0x31
#define TURN_CMD_HAZARD_SET       0x32
#define TURN_DIR_LEFT             0x00
#define TURN_DIR_RIGHT            0x01


 /* =========================
  * 모드 상수 (정환님 기존 유지)
  * ========================= */
static const uint8_t MODE_AUTO = 0;
static const uint8_t MODE_ON = 1;
static const uint8_t MODE_OFF = 2;


/* =========================
 * 방향지시등 모드
 * ========================= */
static const uint8_t TURN_MODE_OFF = 0;
static const uint8_t TURN_MODE_LEFT = 1;
static const uint8_t TURN_MODE_RIGHT = 2;
static const uint8_t TURN_MODE_HAZARD = 3;
/* =========================
 * 속도 레벨 (control 노드와 동일)
 * ========================= */
static const uint8_t kSpeedLevelStop = 0;
static const uint8_t kSpeedLevelSlow = 1;
static const uint8_t kSpeedLevelNormal = 2;
static const uint8_t kSpeedLevelFast = 3;

/* =========================
 * AUTO 임계값 (원하시면 조정)
 * ========================= */
static const float kWiperHumThSlow = 30.0f;
static const float kWiperHumThNormal = 35.0f;
static const float kWiperHumThFast = 40.0f;

/* 수동 ON일 때 고정 speed */
static const uint8_t kManualWiperOnSpeed = kSpeedLevelNormal;

/* 센서가 일정 시간 갱신되지 않으면 AUTO에서 안전상 OFF로 처리 */
static const uint32_t kSensorStaleMs = 6000;  // DHT/온습도
static const uint32_t kCdsStaleMs    = 6000;  // 조도


/* =========================
 * AUTO CDS(조도, ADC raw) threshold
 * - 일반적으로 '어두울수록 ADC가 커지는' 배치가 많음.
 * - 만약 밝을수록 ADC가 커진다면 kCdsDarkIsHighAdc 를 false로 바꾸면 됨.
 * ========================= */
static const bool kCdsDarkIsHighAdc = true;
static const uint16_t kCdsOnThresholdAdc = 800;
static const uint16_t kCdsOffThresholdAdc = 750;
/* =========================
 * 전역 상태
 * ========================= */
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer webServer(80);

static uint8_t g_wiperMode = MODE_AUTO;
static uint8_t g_highBeamMode = MODE_AUTO;


static uint8_t g_turnMode = TURN_MODE_OFF;
static char g_deviceId[16];

static char g_topicWiperCmd[96];
static char g_topicHighBeamCmd[96];
static char g_topicHighCmdCompat[96];
static char g_topicWiperState[96];
static char g_topicHighBeamState[96];
static char g_topicHighStateCompat[96];
static char g_topicOnline[96];

/* 센서 최신값 */
static float g_humidity = NAN;
static float g_temperature = NAN;
static uint32_t g_lastSensorUpdateMs = 0;

static bool g_hasCds = false;
static uint16_t g_cdsAdc = 0;
static uint32_t g_lastCdsUpdateMs = 0;

/* 마지막으로 보낸 명령(중복 송신 줄이기용) */
static uint8_t g_lastSentSpeedLevel = 0xFF;
static uint8_t g_lastSentHighBeamState = 0xFF;

static uint8_t g_lastSentTurnMode = 0xFF;
/* =========================
 * 유틸
 * ========================= */

const char INDEX_HTML[] PROGMEM = R"HTML(
<!doctype html>
<html lang="ko">
<head>
  <meta charset="utf-8" />
  <meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no" />
  <title>ESP32 Control Panel</title>

  <style>
    :root{
      --bg:#050607;
      --neon:#66f3dc;
      --neon-dim:rgba(102,243,220,.45);
      --text:rgba(255,255,255,.78);
      --text-dim:rgba(255,255,255,.55);
      --panel:rgba(0,0,0,.15);
      --shadow:0 0 0 2px var(--neon-dim), 0 0 18px rgba(102,243,220,.10);
      --shadow-strong:0 0 0 2px rgba(102,243,220,.95), 0 0 26px rgba(102,243,220,.28);
      --radius:14px;
      --gap: clamp(10px, 1.6vh, 16px);
      --red:#ff3b30;
    }

    *{ box-sizing:border-box; }
    html, body{
      height:100%;
      margin:0;
      background:var(--bg);
      color:var(--text);
      font-family:system-ui, -apple-system, Segoe UI, Roboto, "Noto Sans KR", sans-serif;
      overflow:hidden;
    }

    .portraitLock{
      display:none;
      position:fixed; inset:0;
      background:rgba(0,0,0,.88);
      color:var(--text);
      align-items:center; justify-content:center;
      text-align:center;
      padding:24px;
      z-index:999;
    }
    @media (orientation: portrait){
      .portraitLock{ display:flex; }
    }

    /* ===== 전체 레이아웃: 좌(2/3) / 우(1/3) ===== */
    .app{
      height:100vh;
      width:100vw;
      padding: clamp(10px, 1.6vh, 16px) clamp(12px, 2vw, 18px);
      display:grid;
      grid-template-columns: 2fr 1fr;
      gap: var(--gap);
    }

    .leftPanel{
      display:grid;
      grid-template-rows: 1fr auto;
      gap: var(--gap);
      min-height:0;
    }

    /* 상단 3개: Wiper / Emergency / High Beam */
    .mainControls{
      display:grid;
      grid-template-columns: repeat(3, 1fr);
      gap: clamp(10px, 1.6vw, 18px);
      align-items:stretch;
      min-height:0;
    }

    /* 하단 2개: Left / Right */
    .turnControls{
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap: clamp(12px, 2vw, 20px);
      align-items:stretch;
      min-height:0;
    }

    .controlGroup{
      display:grid;
      grid-template-rows: 1fr auto;
      gap: clamp(8px, 1.2vh, 12px);
      min-height:0;
    }
    .controlGroup.simple{
      grid-template-rows: 1fr;
    }

    .iconCard{
      width:100%;
      height:100%;
      border-radius:var(--radius);
      background:var(--panel);
      box-shadow:var(--shadow);
      display:grid;
      place-items:center;
      padding: clamp(10px, 1.4vh, 14px);
      user-select:none;
      -webkit-tap-highlight-color:transparent;
      min-height:0;
    }

    .iconCardBtn{
      border:2px solid var(--neon-dim);
      background:transparent;
      cursor:pointer;
      touch-action:manipulation;
    }
    .iconCardBtn.pressed{
      box-shadow:var(--shadow-strong);
      border-color:rgba(102,243,220,.95);
      background:rgba(102,243,220,.08);
      transform: translateY(1px);
    }
    .iconCardBtn.active{
      box-shadow:var(--shadow-strong);
      border-color:rgba(102,243,220,.95);
      background:rgba(102,243,220,.10);
    }

    .iconInner{
      width:100%;
      height:100%;
      display:grid;
      grid-template-rows: 1fr auto;
      align-items:center;
      justify-items:center;
      gap: clamp(8px, 1.2vh, 14px);
    }

    .iconLabel{
      font-size: clamp(14px, 2.2vh, 22px);
      color: rgba(255,255,255,.65);
      letter-spacing:.3px;
      padding-bottom:2px;
    }

    .svgIcon{
      width: clamp(64px, 7.5vw, 130px);
      height:auto;
      max-height:72%;
      opacity:.95;
      filter: drop-shadow(0 0 8px rgba(102,243,220,.18));
    }
    .strokeNeon{ stroke:var(--neon); }
    .fillNeon{ fill:var(--neon); }
    .strokeRed{ stroke:var(--red); }
    .fillRed{ fill:var(--red); }

    .switchStack{
      display:grid;
      grid-template-rows: auto auto;
      gap: clamp(8px, 1.2vh, 12px);
    }
    .segSwitch{
      width:100%;
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap: clamp(8px, 1.2vw, 14px);
    }
    .segBtn, .autoBtn{
      border-radius: 12px;
      border: 2px solid var(--neon-dim);
      background: transparent;
      color: rgba(255,255,255,.70);
      user-select:none;
      -webkit-tap-highlight-color:transparent;
      touch-action: manipulation;
    }
    .segBtn{
      height: clamp(36px, 5.8vh, 52px);
      font-size: clamp(14px, 2.1vh, 20px);
    }
    .autoBtn{
      height: clamp(34px, 5.5vh, 48px);
      font-size: clamp(13px, 2.0vh, 19px);
      letter-spacing: .6px;
    }
    .segBtn.active, .autoBtn.active{
      border-color: rgba(102,243,220,.95);
      box-shadow: var(--shadow-strong);
      background: rgba(102,243,220,.10);
      color: rgba(255,255,255,.92);
    }
    .segBtn:active, .autoBtn:active{ transform: translateY(1px); }

    /* ===== 우측 패널 ===== */
    .rightPanel{
      border-left: 2px solid rgba(102,243,220,.28);
      padding-left: var(--gap);
      display:grid;
      grid-template-rows: auto auto 1fr;
      gap: var(--gap);
      min-height:0;
    }

    .clockBox{
      border-radius: var(--radius);
      background: rgba(0,0,0,.10);
      box-shadow: var(--shadow);
      padding: clamp(12px, 1.6vh, 16px);
    }
    .dateLine{
      font-size: clamp(14px, 2.0vh, 20px);
      color: rgba(255,255,255,.72);
      letter-spacing:.3px;
    }
    .timeLine{
      margin-top: 6px;
      font-size: clamp(20px, 3.2vh, 32px);
      font-weight: 750;
      color: rgba(255,255,255,.92);
      letter-spacing:.6px;
      text-shadow: 0 0 14px rgba(102,243,220,.12);
    }

    .readouts{
      border-radius: var(--radius);
      background: rgba(0,0,0,.10);
      box-shadow: var(--shadow);
      padding: clamp(12px, 1.6vh, 16px);
      font-size: clamp(14px, 2.2vh, 22px);
      line-height: 1.55;
      color: rgba(255,255,255,.70);
    }
    .readouts b{
      color: rgba(255,255,255,.92);
      font-weight: 750;
    }

    .carBox{
      border-radius: var(--radius);
      background: rgba(0,0,0,.10);
      box-shadow: var(--shadow);
      padding: clamp(10px, 1.4vh, 14px);
      display:grid;
      place-items:center;
      min-height:0;
    }

    .carSvg{
      width: 100%;
      max-width: 360px;
      height: auto;
    }

    /* ===== 차량 파트 점멸 ===== */
    @keyframes blink {
      0%   { opacity: 0.20; }
      50%  { opacity: 1.00; }
      100% { opacity: 0.20; }
    }

    .carStroke{
      stroke: var(--neon);
      fill: none;
      stroke-width: 3.5;
      stroke-linejoin: round;
      stroke-linecap: round;
      filter: drop-shadow(0 0 10px rgba(102,243,220,.10));
      opacity: .95;
    }
    .carFillDim{
      fill: rgba(102,243,220,.10);
      stroke: rgba(102,243,220,.35);
      stroke-width: 2;
      filter: drop-shadow(0 0 10px rgba(102,243,220,.08));
    }

    .carPart{
      stroke: rgba(102,243,220,.70);
      fill: none;
      stroke-width: 4;
      stroke-linejoin: round;
      stroke-linecap: round;
      opacity: .95;
      filter: drop-shadow(0 0 12px rgba(102,243,220,.10));
    }

    .carPart.blink{
      stroke: var(--red);
      filter: drop-shadow(0 0 14px rgba(255,59,48,.22));
      animation: blink .65s infinite;
    }
    .carLamp{
      fill: rgba(102,243,220,.35);
      stroke: rgba(102,243,220,.70);
      stroke-width: 3;
      opacity: .95;
      filter: drop-shadow(0 0 14px rgba(102,243,220,.10));
    }
    .carLamp.blink{
      fill: rgba(255,59,48,.85);
      stroke: var(--red);
      filter: drop-shadow(0 0 14px rgba(255,59,48,.24));
      animation: blink .65s infinite;
    }
  </style>
</head>

<body>
  <div class="portraitLock">
    <div>
      <div style="font-size:22px; margin-bottom:10px; color: rgba(255,255,255,.85);">가로 모드에서 사용하십시오.</div>
      <div style="font-size:14px; color: rgba(255,255,255,.55);">태블릿을 가로로 눕히면 컨트롤 패널이 표시됩니다.</div>
    </div>
  </div>

  <div class="app">

    <!-- ===== 좌측(2/3): 버튼 ===== -->
    <div class="leftPanel">

      <div class="mainControls">

        <!-- WIPER -->
        <div class="controlGroup" data-group="wiper">
          <div class="iconCard">
            <div class="iconInner">
              <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linejoin="round"
                      d="M12 16 Q14 10 20 10 H44 Q50 10 52 16 L56 34
                         Q57 40 53 44 Q49 48 42 46 Q32 44 22 46
                         Q15 48 11 44 Q7 40 8 34 L12 16Z"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linejoin="round"
                      d="M14 18 L22 14 L17 28 Z" opacity="0.55"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                      d="M28 22 V52"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                      d="M36 22 V52"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                      d="M28 34 L24 32"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                      d="M36 34 L40 32"/>
                <circle class="fillNeon" cx="28" cy="54" r="3.6"/>
                <circle class="fillNeon" cx="36" cy="54" r="3.6"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                      d="M40 20 C46 20 50 23 54 27"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                      d="M40 28 C45 28 48 30 51 33"/>
              </svg>
              <div class="iconLabel">Wiper</div>
            </div>
          </div>

          <div class="switchStack">
            <div class="segSwitch">
              <button class="segBtn active" data-seg="on" type="button">On</button>
              <button class="segBtn" data-seg="off" type="button">Off</button>
            </div>
            <button class="autoBtn" data-seg="auto" type="button">AUTO</button>
          </div>
        </div>

        <!-- EMERGENCY -->
        <div class="controlGroup simple" data-group="emer">
          <button class="iconCard iconCardBtn" type="button" data-box="emer" aria-label="Emergency">
            <div class="iconInner">
              <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
                <path class="strokeRed" fill="none" stroke-width="5" stroke-linejoin="round"
                      d="M32 10 L56 52 H8 L32 10 Z"/>
                <path class="strokeRed" fill="none" stroke-width="5" stroke-linejoin="round" opacity="0.85"
                      d="M32 20 L46 46 H18 L32 20 Z"/>
              </svg>
              <div class="iconLabel">Emergency</div>
            </div>
          </button>
        </div>

        <!-- HIGH BEAM -->
        <div class="controlGroup" data-group="high_beam">
          <div class="iconCard">
            <div class="iconInner">
              <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round" d="M10 24 H26"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round" d="M10 32 H26"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round" d="M10 40 H26"/>
                <path class="strokeNeon" fill="none" stroke-width="4" stroke-linejoin="round"
                      d="M28 18 H42 Q54 18 54 32 Q54 46 42 46 H28 Z"/>
              </svg>
              <div class="iconLabel">High Beam</div>
            </div>
          </div>

          <div class="switchStack">
            <div class="segSwitch">
              <button class="segBtn active" data-seg="on" type="button">On</button>
              <button class="segBtn" data-seg="off" type="button">Off</button>
            </div>
            <button class="autoBtn" data-seg="auto" type="button">AUTO</button>
          </div>
        </div>

      </div>

      <div class="turnControls">
        <!-- LEFT -->
        <button class="iconCard iconCardBtn" type="button" data-box="left" aria-label="Left">
          <div class="iconInner">
            <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
              <path class="fillNeon" d="M28 12 8 32l20 20v-12h28V24H28V12z"/>
            </svg>
            <div class="iconLabel">Left</div>
          </div>
        </button>

        <!-- RIGHT -->
        <button class="iconCard iconCardBtn" type="button" data-box="right" aria-label="Right">
          <div class="iconInner">
            <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
              <path class="fillNeon" d="M36 12 56 32 36 52V40H8V24h28V12z"/>
            </svg>
            <div class="iconLabel">Right</div>
          </div>
        </button>
      </div>

    </div>

    <!-- ===== 우측(1/3): 날짜/시간 + 센서 + 차량 ===== -->
    <div class="rightPanel">

      <div class="clockBox" aria-label="Date and Time">
        <div class="dateLine" id="dateText">----</div>
        <div class="timeLine" id="timeText">--:--:--</div>
      </div>

      <div class="readouts" aria-label="Sensor Readouts">
        <div>온도 : <b id="tempText">--°C</b></div>
        <div>습도 : <b id="humText">--%</b></div>
        <div>현재 밝기 : <b id="cdsText">--</b></div>
      </div>

      <div class="carBox" aria-label="Car Status">
        <!-- 위에서 바라본 차량(앞쪽이 위) -->
        <svg class="carSvg" viewBox="0 0 240 420" aria-hidden="true">
          <!-- Car body -->
          <path class="carStroke"
                d="M70 40
                   Q120 10 170 40
                   Q186 52 190 80
                   L200 170
                   Q205 200 205 220
                   L205 300
                   Q205 332 188 352
                   Q170 372 140 374
                   L100 374
                   Q70 372 52 352
                   Q35 332 35 300
                   L35 220
                   Q35 200 40 170
                   L50 80
                   Q54 52 70 40 Z"/>

          <!-- cabin / glass outline -->
          <path id="windshield" class="carPart"
                d="M78 90
                   Q120 66 162 90
                   L175 170
                   Q178 190 165 205
                   Q150 222 120 218
                   Q90 222 75 205
                   Q62 190 65 170
                   L78 90 Z"/>

          <!-- roof/cabin fill -->
          <path class="carFillDim"
                d="M88 108
                   Q120 88 152 108
                   L162 164
                   Q165 178 156 188
                   Q142 204 120 200
                   Q98 204 84 188
                   Q75 178 78 164
                   L88 108 Z"/>

          <!-- headlamps (front is top) -->
          <path id="headL" class="carLamp"
                d="M62 70 Q72 56 92 58 Q92 74 76 82 Q60 88 52 86 Q54 78 62 70 Z"/>
          <path id="headR" class="carLamp"
                d="M178 70 Q168 56 148 58 Q148 74 164 82 Q180 88 188 86 Q186 78 178 70 Z"/>

          <!-- turn signals (front) -->
          <circle id="turnL" class="carLamp" cx="52" cy="108" r="10"/>
          <circle id="turnR" class="carLamp" cx="188" cy="108" r="10"/>

          <!-- rear lights (dim only) -->
          <rect class="carLamp" x="54" y="332" width="34" height="16" rx="8" opacity="0.45"/>
          <rect class="carLamp" x="152" y="332" width="34" height="16" rx="8" opacity="0.45"/>

          <!-- center line -->
          <path class="carStroke" opacity="0.35"
                d="M120 70 L120 350"/>
        </svg>
      </div>
    </div>
  </div>

<script>
  // ===== 공용 유틸 =====
  function setActive(element, on) {
    if (!element) return;
    element.classList.toggle('active', Boolean(on));
  }

  function toLowerSafe(v){
    if (typeof v !== 'string') return '';
    return v.toLowerCase();
  }

  // ===== 시간 표시(로컬) =====
  function tickClock(){
    const now = new Date();
    const dateStr = now.toLocaleDateString('ko-KR', { year:'numeric', month:'2-digit', day:'2-digit', weekday:'short' });
    const timeStr = now.toLocaleTimeString('ko-KR', { hour:'2-digit', minute:'2-digit', second:'2-digit' });
    const dateEl = document.getElementById('dateText');
    const timeEl = document.getElementById('timeText');
    if (dateEl) dateEl.textContent = dateStr;
    if (timeEl) timeEl.textContent = timeStr;
  }
  tickClock();
  setInterval(tickClock, 1000);

  // ===== ESP32로 명령 보내기 =====
  async function sendMode(target, state) {
    // state: 'auto' | 'on' | 'off'
    let mode = 'AUTO';
    if (state === 'on') mode = 'ON';
    if (state === 'off') mode = 'OFF';

    try {
      await fetch('/api/set?target=' + encodeURIComponent(target) + '&mode=' + encodeURIComponent(mode));
    } catch (e) {}
  }

  async function sendTurn(mode) {
    // mode: 'OFF' | 'LEFT' | 'RIGHT' | 'HAZARD'
    try {
      await fetch('/api/turn?mode=' + encodeURIComponent(mode));
    } catch (e) {}
  }

  // ===== 차량 점멸 제어 =====
  function setBlinkById(id, on){
    const el = document.getElementById(id);
    if (!el) return;
    el.classList.toggle('blink', Boolean(on));
  }

  function updateCarBlink(state){
    // state: { turn:'off'|'left'|'right'|'hazard', wiperActive:boolean, highActive:boolean }
    const turn = state.turn || 'off';
    const isLeft = (turn === 'left' || turn === 'hazard');
    const isRight = (turn === 'right' || turn === 'hazard');

    setBlinkById('turnL', isLeft);
    setBlinkById('turnR', isRight);

    setBlinkById('windshield', Boolean(state.wiperActive));
    setBlinkById('headL', Boolean(state.highActive));
    setBlinkById('headR', Boolean(state.highActive));
  }

  // ===== UI 그룹 상태 반영(세그/오토) =====
  function setGroupUI(groupEl, state) {
    const segmentButtons = groupEl.querySelectorAll('.segBtn');
    const autoButton = groupEl.querySelector('.autoBtn');
    if (!autoButton) return;

    segmentButtons.forEach(button => button.classList.remove('active'));
    autoButton.classList.remove('active');

    if (state === 'auto') {
      autoButton.classList.add('active');
      return;
    }

    segmentButtons.forEach(button => {
      if (button.dataset.seg === state) {
        button.classList.add('active');
      }
    });
  }

  // ===== 버튼 참조 =====
  const leftButton  = document.querySelector('[data-box="left"]');
  const rightButton = document.querySelector('[data-box="right"]');
  const emerButton  = document.querySelector('[data-box="emer"]');

  function isHazardOn() {
    if (!emerButton) return false;
    return emerButton.classList.contains('active');
  }

  function setTurnSignal(side) {
    if (!leftButton || !rightButton) return;

    if (side === 'left') {
      setActive(leftButton, true);
      setActive(rightButton, false);
      return;
    }

    if (side === 'right') {
      setActive(leftButton, false);
      setActive(rightButton, true);
      return;
    }

    setActive(leftButton, false);
    setActive(rightButton, false);
  }

  function setHazard(on) {
    setActive(emerButton, on);
    setActive(leftButton, on);
    setActive(rightButton, on);
  }

  // ===== 로컬 UI 기준 차량 즉시 반영(서버 응답 오기 전) =====
  function updateCarFromUI(){
    const isHaz = isHazardOn();
    const lOn = leftButton ? leftButton.classList.contains('active') : false;
    const rOn = rightButton ? rightButton.classList.contains('active') : false;

    let turn = 'off';
    if (isHaz) {
      turn = 'hazard';
    } else if (lOn) {
      turn = 'left';
    } else if (rOn) {
      turn = 'right';
    }

    const wiperGroup = document.querySelector('.controlGroup[data-group="wiper"]');
    const highGroup  = document.querySelector('.controlGroup[data-group="high_beam"]');

    // "실제 동작" 판단은 서버가 active 플래그를 주면 그걸 써야 정확함.
    // 일단 UI가 ON이면 점멸하도록 처리.
    let wiperActive = false;
    if (wiperGroup) {
      const onBtn = wiperGroup.querySelector('.segBtn[data-seg="on"]');
      if (onBtn) wiperActive = onBtn.classList.contains('active');
    }

    let highActive = false;
    if (highGroup) {
      const onBtn = highGroup.querySelector('.segBtn[data-seg="on"]');
      if (onBtn) highActive = onBtn.classList.contains('active');
    }

    updateCarBlink({ turn, wiperActive, highActive });
  }

  // ===== 세그 버튼 동작 + 서버 전송 =====
  document.querySelectorAll('.controlGroup').forEach(group => {
    const groupName = group.dataset.group || '';
    const segmentButtons = group.querySelectorAll('.segBtn');
    const autoButton = group.querySelector('.autoBtn');

    if (!autoButton) return;

    function clearAll() {
      segmentButtons.forEach(button => button.classList.remove('active'));
      autoButton.classList.remove('active');
    }

    async function setState(state) {
      clearAll();

      if (state === 'auto') {
        autoButton.classList.add('active');
      } else {
        segmentButtons.forEach(button => {
          if (button.dataset.seg === state) {
            button.classList.add('active');
          }
        });
      }

      if (groupName === 'wiper' || groupName === 'high_beam') {
        await sendMode(groupName, state);
      }

      updateCarFromUI();
    }

    segmentButtons.forEach(button => {
      button.addEventListener('click', () => setState(button.dataset.seg));
    });
    autoButton.addEventListener('click', () => setState('auto'));

    // 기본값 AUTO
    clearAll();
    autoButton.classList.add('active');
  });

  // ===== Left/Right/Emergency 클릭 =====
  if (leftButton) {
    leftButton.addEventListener('click', async () => {
      if (isHazardOn()) {
        setHazard(false);
        await sendTurn('OFF');
      }

      const wantOn = !leftButton.classList.contains('active');
      setTurnSignal(wantOn ? 'left' : 'off');
      await sendTurn(wantOn ? 'LEFT' : 'OFF');

      updateCarFromUI();
    });
  }

  if (rightButton) {
    rightButton.addEventListener('click', async () => {
      if (isHazardOn()) {
        setHazard(false);
        await sendTurn('OFF');
      }

      const wantOn = !rightButton.classList.contains('active');
      setTurnSignal(wantOn ? 'right' : 'off');
      await sendTurn(wantOn ? 'RIGHT' : 'OFF');

      updateCarFromUI();
    });
  }

  if (emerButton) {
    emerButton.addEventListener('click', async () => {
      const nextOn = !emerButton.classList.contains('active');
      setHazard(nextOn);
      await sendTurn(nextOn ? 'HAZARD' : 'OFF');

      updateCarFromUI();
    });
  }

  // ===== pressed 효과 =====
  document.querySelectorAll('.iconCardBtn').forEach(button => {
    button.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      button.classList.add('pressed');
    });

    ['pointerup', 'pointercancel', 'pointerleave'].forEach(eventName => {
      button.addEventListener(eventName, () => button.classList.remove('pressed'));
    });
  });

  // ===== 서버 상태 동기화 =====
  function normalizeMode(value){
    const s = toLowerSafe(value);
    if (s === 'on' || s === 'off' || s === 'auto') return s;
    if (s === '1') return 'on';
    if (s === '0') return 'off';
    return 'auto';
  }

  function pickBool(obj, keys){
    for (const key of keys){
      if (obj && Object.prototype.hasOwnProperty.call(obj, key)){
        return Boolean(obj[key]);
      }
    }
    return null;
  }

  async function refreshFromServer(){
    try{
      const r = await fetch('/api/state');
      const j = await r.json();

      // wiper/high_beam UI
      const wiperGroup = document.querySelector('.controlGroup[data-group="wiper"]');
      const highGroup  = document.querySelector('.controlGroup[data-group="high_beam"]');

      let wiperMode = 'auto';
      let highMode = 'auto';

      if (wiperGroup) {
        wiperMode = normalizeMode(j.wiper || j.wiper_mode || j.wiperMode || 'AUTO');
        setGroupUI(wiperGroup, wiperMode);
      }

      if (highGroup) {
        highMode = normalizeMode(j.high_beam || j.high || j.high_mode || j.highMode || 'AUTO');
        setGroupUI(highGroup, highMode);
      }

      // turn UI
      let turn = 'off';
      if (typeof j.turn === 'string') {
        const t = toLowerSafe(j.turn || 'OFF');
        if (t === 'hazard' || t === 'emergency') {
          turn = 'hazard';
          setHazard(true);
        } else {
          setHazard(false);
          if (t === 'left') {
            turn = 'left';
            setTurnSignal('left');
          } else if (t === 'right') {
            turn = 'right';
            setTurnSignal('right');
          } else {
            turn = 'off';
            setTurnSignal('off');
          }
        }
      }

      // 센서 표시
      if (typeof j.temp === 'number') {
        document.getElementById('tempText').textContent = j.temp.toFixed(1) + '°C';
      }
      if (typeof j.hum === 'number') {
        document.getElementById('humText').textContent = j.hum.toFixed(1) + '%';
      }
      if (typeof j.cds === 'number') {
        document.getElementById('cdsText').textContent = String(j.cds);
      }

      // "실제 동작" 점멸 플래그(있으면 사용, 없으면 모드=ON을 fallback)
      const wiperActiveFromServer = pickBool(j, ['wiper_active','wiperActive','wiper_running','wiperRunning','wiper_on','wiperOn']);
      const highActiveFromServer  = pickBool(j, ['high_active','highActive','high_running','highRunning','high_on','highOn','high_beam_on','highBeamOn']);

      const wiperActive = (wiperActiveFromServer !== null) ? wiperActiveFromServer : (wiperMode === 'on');
      const highActive  = (highActiveFromServer  !== null) ? highActiveFromServer  : (highMode  === 'on');

      updateCarBlink({ turn, wiperActive, highActive });

    } catch(e) {
      // 네트워크 실패 시: 로컬 UI 기준으로라도 갱신
      updateCarFromUI();
    }
  }

  refreshFromServer();
  setInterval(refreshFromServer, 1000);
</script>

</body>
</html>
)HTML";

static const char* mode_to_string(uint8_t mode)
{
    if (mode == MODE_AUTO) {
        return "AUTO";
    }
    if (mode == MODE_ON) {
        return "ON";
    }
    if (mode == MODE_OFF) {
        return "OFF";
    }
    return "UNKNOWN";
}

static const char* turn_mode_to_string(uint8_t mode)
{
    if (mode == TURN_MODE_OFF) {
        return "OFF";
    }
    if (mode == TURN_MODE_LEFT) {
        return "LEFT";
    }
    if (mode == TURN_MODE_RIGHT) {
        return "RIGHT";
    }
    if (mode == TURN_MODE_HAZARD) {
        return "HAZARD";
    }
    return "UNKNOWN";
}

static bool parse_turn_mode_query(const char* modeStr, uint8_t* outMode)
{
    if (modeStr == NULL) {
        return false;
    }
    if (outMode == NULL) {
        return false;
    }

    if (strcmp(modeStr, "OFF") == 0) {
        *outMode = TURN_MODE_OFF;
        return true;
    }
    if (strcmp(modeStr, "LEFT") == 0) {
        *outMode = TURN_MODE_LEFT;
        return true;
    }
    if (strcmp(modeStr, "RIGHT") == 0) {
        *outMode = TURN_MODE_RIGHT;
        return true;
    }
    if (strcmp(modeStr, "HAZARD") == 0) {
        *outMode = TURN_MODE_HAZARD;
        return true;
    }
    return false;
}

static bool parse_mode_payload(const char* payload, uint8_t* outMode)
{
    if (payload == NULL) {
        return false;
    }
    if (outMode == NULL) {
        return false;
    }

    if (strcmp(payload, "AUTO") == 0) {
        *outMode = MODE_AUTO;
        return true;
    }

    if (strcmp(payload, "ON") == 0) {
        *outMode = MODE_ON;
        return true;
    }

    if (strcmp(payload, "OFF") == 0) {
        *outMode = MODE_OFF;
        return true;
    }

    return false;
}

/* =========================
 * CAN init / send / receive
 * ========================= */
static bool canInit()
{
    twai_general_config_t generalConfig =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_GPIO_PIN, CAN_RX_GPIO_PIN, TWAI_MODE_NORMAL);

    twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
    twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t result = twai_driver_install(&generalConfig, &timingConfig, &filterConfig);
    if (result != ESP_OK) {
        Serial.printf("CAN install fail: %d\n", (int)result);
        return false;
    }

    result = twai_start();
    if (result != ESP_OK) {
        Serial.printf("CAN start fail: %d\n", (int)result);
        return false;
    }

    Serial.println("CAN start OK");
    return true;
}

static bool can_send_servo_speed(uint8_t speedLevel)
{
    twai_message_t tx = {};
    tx.identifier = CAN_ID_CMD_SERVO;
    tx.flags = TWAI_MSG_FLAG_NONE;
    tx.data_length_code = 2;
    tx.data[0] = SERVO_CMD_SET_SPEED_LEVEL;
    tx.data[1] = speedLevel;

    esp_err_t result = twai_transmit(&tx, 10);
    if (result != ESP_OK) {
        Serial.printf("[CAN-TX-FAIL] SERVO id=0x%03X err=%d speed=%u\n",
            (unsigned)tx.identifier,
            (int)result,
            (unsigned)speedLevel);
        return false;
    }

    return true;
}


static bool can_send_high_beam_state(uint8_t highBeamState)
{
    uint8_t onoff = (highBeamState != 0U) ? 1U : 0U;

    twai_message_t tx = {};
    tx.identifier = CAN_ID_CMD_HIGH_BEAM;
    tx.flags = TWAI_MSG_FLAG_NONE;
    tx.data_length_code = 2;
    tx.data[0] = HIGH_BEAM_CMD_SET_STATE;
    tx.data[1] = onoff;

    esp_err_t result = twai_transmit(&tx, 10);
    if (result != ESP_OK) {
        Serial.printf("[CAN-TX-FAIL] HIGH_BEAM id=0x%03X err=%d state=%u\n",
            (unsigned)tx.identifier,
            (int)result,
            (unsigned)onoff);
        return false;
    }

    return true;
}

static bool can_send_turn_pulse(uint8_t dir)
{
    twai_message_t tx = {};
    tx.identifier = CAN_ID_CMD_TURN;
    tx.flags = TWAI_MSG_FLAG_NONE;
    tx.data_length_code = 2;
    tx.data[0] = TURN_CMD_PULSE;
    tx.data[1] = dir;

    esp_err_t result = twai_transmit(&tx, 10);
    if (result != ESP_OK) {
        Serial.printf("[CAN-TX-FAIL] TURN_PULSE id=0x%03X err=%d dir=%u\n",
            (unsigned)tx.identifier, (int)result, (unsigned)dir);
        return false;
    }
    return true;
}

static bool can_send_hazard_set(bool enabled)
{
    twai_message_t tx = {};
    tx.identifier = CAN_ID_CMD_TURN;
    tx.flags = TWAI_MSG_FLAG_NONE;
    tx.data_length_code = 2;
    tx.data[0] = TURN_CMD_HAZARD_SET;
    tx.data[1] = enabled ? 1U : 0U;

    esp_err_t result = twai_transmit(&tx, 10);
    if (result != ESP_OK) {
        Serial.printf("[CAN-TX-FAIL] HAZARD_SET id=0x%03X err=%d en=%u\n",
            (unsigned)tx.identifier, (int)result, enabled ? 1U : 0U);
        return false;
    }
    return true;
}

static bool send_turn_transition(uint8_t prevMode, uint8_t newMode)
{
    bool ok = true;

    // prevMode가 초기값(0xFF)일 때는 의미 없는 OFF 처리 생략
    if (prevMode == 0xFF) {
        if (newMode == TURN_MODE_HAZARD) {
            return can_send_hazard_set(true);
        }
        if (newMode == TURN_MODE_LEFT) {
            return can_send_turn_pulse(TURN_DIR_LEFT);
        }
        if (newMode == TURN_MODE_RIGHT) {
            return can_send_turn_pulse(TURN_DIR_RIGHT);
        }
        return true; // newMode == OFF
    }

    // hazard -> 다른 모드: hazard 먼저 끔
    if (prevMode == TURN_MODE_HAZARD && newMode != TURN_MODE_HAZARD) {
        ok = can_send_hazard_set(false) && ok;
    }

    // new hazard: hazard 켬
    if (newMode == TURN_MODE_HAZARD) {
        ok = can_send_hazard_set(true) && ok;
        return ok;
    }

    // new left/right: 한 번만 PULSE 보내면 CONTROL이 알아서 점멸 시작/방향전환 처리함
    if (newMode == TURN_MODE_LEFT) {
        ok = can_send_turn_pulse(TURN_DIR_LEFT) && ok;
        return ok;
    }

    if (newMode == TURN_MODE_RIGHT) {
        ok = can_send_turn_pulse(TURN_DIR_RIGHT) && ok;
        return ok;
    }

    // new OFF:
    // 기존에 LEFT/RIGHT였으면 같은 방향 PULSE 한 번 더 보내서 "토글 OFF" 시킴
    if (prevMode == TURN_MODE_LEFT) {
        ok = can_send_turn_pulse(TURN_DIR_LEFT) && ok;
        return ok;
    }

    if (prevMode == TURN_MODE_RIGHT) {
        ok = can_send_turn_pulse(TURN_DIR_RIGHT) && ok;
        return ok;
    }

    if (prevMode == TURN_MODE_HAZARD) {
        ok = can_send_hazard_set(false) && ok;
        return ok;
    }

    return true;
}


static void canPollSensors()
{
    for (;;)
    {
        twai_message_t rxMessage;
        esp_err_t result = twai_receive(&rxMessage, 0);
        if (result != ESP_OK) {
            break;
        }

        /* 이 프로젝트는 STD ID만 사용 */
        if ((rxMessage.flags & TWAI_MSG_FLAG_EXTD) != 0) {
            continue;
        }

        /* =========================
         * DHT (CAN_DHT_ID = 0x100)
         * - 지원 포맷(자동 판별):
         *   1) DLC=8: float humidity(4B) + float temp(4B)  [LE]
         *   2) DLC>=4: int16 humidity_x10(2B) + int16 temp_x10(2B) [LE]
         *   3) DLC=2: uint8 humidity + uint8 temp
         * ========================= */
        if (rxMessage.identifier == CAN_DHT_ID)
        {
            bool updated = false;

            if (rxMessage.data_length_code == 8) {
                float humidity = NAN;
                float temperature = NAN;

                memcpy(&humidity, &rxMessage.data[0], 4);
                memcpy(&temperature, &rxMessage.data[4], 4);

                if (isfinite(humidity) && isfinite(temperature)) {
                    g_humidity = humidity;
                    g_temperature = temperature;
                    updated = true;
                }
            }
            else if (rxMessage.data_length_code >= 4) {
                int16_t humidity_x10 = (int16_t)((uint16_t)rxMessage.data[0] | ((uint16_t)rxMessage.data[1] << 8));
                int16_t temp_x10     = (int16_t)((uint16_t)rxMessage.data[2] | ((uint16_t)rxMessage.data[3] << 8));

                float humidity = ((float)humidity_x10) / 10.0f;
                float temperature = ((float)temp_x10) / 10.0f;

                /* 값이 말이 되면 적용(송신 측 포맷 혼선 방지) */
                bool humidity_ok = (humidity >= 0.0f) && (humidity <= 100.0f);
                bool temperature_ok = (temperature >= -40.0f) && (temperature <= 125.0f);
                if (humidity_ok && temperature_ok) {
                    g_humidity = humidity;
                    g_temperature = temperature;
                    updated = true;
                }
            }
            else if (rxMessage.data_length_code == 2) {
                uint8_t humidity = rxMessage.data[0];
                uint8_t temperature = rxMessage.data[1];

                g_humidity = (float)humidity;
                g_temperature = (float)temperature;
                updated = true;
            }

            if (updated) {
                g_lastSensorUpdateMs = millis();
            }
            continue;
        }

        /* =========================
         * CDS (CAN_CDS_ID = 0x110)
         * - 지원 포맷:
         *   1) DLC>=2: uint16 adc [LE]
         *   2) DLC=1 : uint8 adc
         * ========================= */
        if (rxMessage.identifier == CAN_CDS_ID)
        {
            if (rxMessage.data_length_code >= 2) {
                uint16_t cdsAdc12 = (uint16_t)((uint16_t)rxMessage.data[0] | ((uint16_t)rxMessage.data[1] << 8));
                g_cdsAdc = cdsAdc12;
                g_hasCds = true;
                g_lastCdsUpdateMs = millis();
                continue;
            }

            if (rxMessage.data_length_code == 1) {
                g_cdsAdc = (uint16_t)rxMessage.data[0];
                g_hasCds = true;
                g_lastCdsUpdateMs = millis();
                continue;
            }

            continue;
        }
    }
}

/* =========================
 * MQTT publish
 * ========================= */
static void mqtt_publish_state()
{
    mqttClient.publish(g_topicWiperState, mode_to_string(g_wiperMode), true);
    mqttClient.publish(g_topicHighBeamState, mode_to_string(g_highBeamMode), true);
    mqttClient.publish(g_topicHighStateCompat, mode_to_string(g_highBeamMode), true);
}

/* =========================
 * 모드 적용 (SPI 제거, CAN 명령은 loop에서 처리)
 * ========================= */
static void apply_wiper_mode(uint8_t newMode)
{
    if (g_wiperMode == newMode) {
        return;
    }

    g_wiperMode = newMode;
    Serial.printf("[APPLY] WIPER=%s\n", mode_to_string(g_wiperMode));
    mqtt_publish_state();

    /* 모드 바뀌면 즉시 재송신 유도 */
    g_lastSentSpeedLevel = 0xFF;
}

static void apply_high_beam_mode(uint8_t newMode)
{
    if (g_highBeamMode == newMode) {
        return;
    }

    g_highBeamMode = newMode;
    Serial.printf("[APPLY] HIGH_BEAM=%s\n", mode_to_string(g_highBeamMode));
    mqtt_publish_state();

    g_lastSentHighBeamState = 0xFF;
}


static void apply_turn_mode(uint8_t newMode)
{
    if (newMode > TURN_MODE_HAZARD) {
        newMode = TURN_MODE_HAZARD;
    }

    if (g_turnMode == newMode) {
        return;
    }

    g_turnMode = newMode;
    Serial.printf("[APPLY] TURN=%s", turn_mode_to_string(g_turnMode));

    /* 모드 바뀌면 즉시 재송신 유도 */
    g_lastSentTurnMode = 0xFF;
}

/* =========================
 * MQTT callback
 * ========================= */
static void mqtt_callback(char* topic, byte* payload, unsigned int length)
{
    if (topic == NULL) {
        return;
    }
    if (payload == NULL) {
        return;
    }

    char message[32];
    unsigned int copyLength = length;

    if (copyLength >= sizeof(message)) {
        copyLength = sizeof(message) - 1;
    }

    memcpy(message, payload, copyLength);
    message[copyLength] = '\0';

    for (unsigned int index = 0; index < copyLength; index++) {
        char c = message[index];
        if (c >= 'a' && c <= 'z') {
            message[index] = (char)(c - 'a' + 'A');
        }
    }

    uint8_t newMode = MODE_AUTO;
    Serial.printf("[MQTT-RX] topic=%s payload=%s len=%u ms=%lu\n",
        topic, message, (unsigned)length, (unsigned long)millis());

    bool ok = parse_mode_payload(message, &newMode);
    if (ok == false) {
        Serial.printf("[MQTT] invalid payload: %s\n", message);
        return;
    }

    if (strcmp(topic, g_topicWiperCmd) == 0) {
        Serial.printf("[MQTT] WIPER=%s\n", mode_to_string(newMode));
        apply_wiper_mode(newMode);
        return;
    }

    if (strcmp(topic, g_topicHighBeamCmd) == 0 || strcmp(topic, g_topicHighCmdCompat) == 0) {
        Serial.printf("[MQTT] HIGH_BEAM=%s\n", mode_to_string(newMode));
        apply_high_beam_mode(newMode);
        return;
    }
}

/* =========================
 * 토픽 생성 (정환님 기존 유지)
 * ========================= */
static void build_device_id_from_mac()
{
    uint64_t mac = ESP.getEfuseMac();
    uint32_t low24 = (uint32_t)(mac & 0xFFFFFF);
    snprintf(g_deviceId, sizeof(g_deviceId), "%06X", (unsigned)low24);
}

static void build_topics()
{
    snprintf(g_topicWiperCmd, sizeof(g_topicWiperCmd), "%s/wiper/cmd/%s", TOPIC_BASE, g_deviceId);
    snprintf(g_topicHighBeamCmd, sizeof(g_topicHighBeamCmd), "%s/high_beam/cmd/%s", TOPIC_BASE, g_deviceId);
    snprintf(g_topicHighCmdCompat, sizeof(g_topicHighCmdCompat), "%s/high/cmd/%s", TOPIC_BASE, g_deviceId);

    snprintf(g_topicWiperState, sizeof(g_topicWiperState), "%s/wiper/state/%s", TOPIC_BASE, g_deviceId);
    snprintf(g_topicHighBeamState, sizeof(g_topicHighBeamState), "%s/high_beam/state/%s", TOPIC_BASE, g_deviceId);
    snprintf(g_topicHighStateCompat, sizeof(g_topicHighStateCompat), "%s/high/state/%s", TOPIC_BASE, g_deviceId);

    snprintf(g_topicOnline, sizeof(g_topicOnline), "%s/online/%s", TOPIC_BASE, g_deviceId);

    Serial.println("[TOPICS]");
    Serial.println(g_topicWiperCmd);
    Serial.println(g_topicHighBeamCmd);
    Serial.println(g_topicHighCmdCompat);
    Serial.println(g_topicWiperState);
    Serial.println(g_topicHighBeamState);
    Serial.println(g_topicHighStateCompat);
    Serial.println(g_topicOnline);
}

/* =========================
 * WiFi/MQTT connect 
 * ========================= */
static void wifi_connect()
{
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFI_SSID, WIFI_PASS);

    Serial.print("WiFi connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(300);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
}

static void mqtt_connect()
{
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    #if ENABLE_MQTT_COMMANDS
    mqttClient.setCallback(mqtt_callback);
#endif

    while (mqttClient.connected() == false)
    {
        String clientId = "esp32-mqtt-";
        clientId += g_deviceId;

        bool connected = mqttClient.connect(
            clientId.c_str(),
            g_topicOnline,
            1,
            true,
            "offline"
        );

        if (connected) {
            Serial.println("[MQTT] CONNECTED");
            Serial.print("[MQTT] clientId="); Serial.println(clientId);
            Serial.print("[MQTT] broker="); Serial.print(MQTT_HOST);
            Serial.print(":"); Serial.println(MQTT_PORT);
            break;
        }

        Serial.print("[MQTT] connect fail, state=");
        Serial.println(mqttClient.state());
        delay(500);
    }

#if ENABLE_MQTT_COMMANDS
    mqttClient.subscribe(g_topicWiperCmd);
    mqttClient.subscribe(g_topicHighBeamCmd);
    mqttClient.subscribe(g_topicHighCmdCompat);
#endif

    mqttClient.publish(g_topicOnline, "online", true);
    mqtt_publish_state();
}

/* =========================
 * Web API 
 * ========================= */
static void web_handle_root()
{
    webServer.send_P(200, "text/html", INDEX_HTML);
}

static void web_handle_state()
{
    String json = "{";
    json += "\"deviceId\":\"";
    json += g_deviceId;
    json += "\",";

    json += "\"topicBase\":\"";
    json += TOPIC_BASE;
    json += "\",";

    json += "\"wiper\":\"";
    json += mode_to_string(g_wiperMode);
    json += "\",";

    /* 호환: high(구버전) + high_beam(신버전) 둘 다 제공 */
    json += "\"high_beam\":\"";
    json += mode_to_string(g_highBeamMode);
    json += "\",";

    json += "\"high\":\"";
    json += mode_to_string(g_highBeamMode);
    json += "\",";

    json += "\"turn\":\"";
    json += turn_mode_to_string(g_turnMode);
    json += "\"";

    /* 센서 값 */
    if (isnan(g_temperature) == false) {
        json += ",\"temp\":";
        json += String(g_temperature, 1);
    }

    if (isnan(g_humidity) == false) {
        json += ",\"hum\":";
        json += String(g_humidity, 1);
    }

    if (g_hasCds) {
        json += ",\"cds\":";
        json += String((unsigned)g_cdsAdc);
    }

    /* 액추에이터 현재(마지막 송신) 상태: UI 점멸 용 */
    bool wiperActive = (g_lastSentSpeedLevel != 0xFF) && (g_lastSentSpeedLevel != kSpeedLevelStop);
    bool highActive  = (g_lastSentHighBeamState != 0xFF) && (g_lastSentHighBeamState != 0);

    json += ",\"wiper_active\":";
    json += (wiperActive ? "true" : "false");
    json += ",\"high_active\":";
    json += (highActive ? "true" : "false");

    if (g_lastSentSpeedLevel != 0xFF) {
        json += ",\"wiper_speed_level\":";
        json += String((unsigned)g_lastSentSpeedLevel);
    }
    if (g_lastSentHighBeamState != 0xFF) {
        json += ",\"high_state\":";
        json += String((unsigned)g_lastSentHighBeamState);
    }

    /* 센서 갱신 시각(디버그용) */
    uint32_t nowMs = millis();
    if (g_lastSensorUpdateMs != 0) {
        json += ",\"dht_age_ms\":";
        json += String((unsigned)(nowMs - g_lastSensorUpdateMs));
    }
    if (g_lastCdsUpdateMs != 0) {
        json += ",\"cds_age_ms\":";
        json += String((unsigned)(nowMs - g_lastCdsUpdateMs));
    }

    json += "}";
    webServer.send(200, "application/json", json);
}


static void web_handle_set()
{
    if (webServer.hasArg("target") == false) {
        webServer.send(400, "text/plain", "missing target");
        return;
    }

    if (webServer.hasArg("mode") == false) {
        webServer.send(400, "text/plain", "missing mode");
        return;
    }

    String target = webServer.arg("target");
    String modeStr = webServer.arg("mode");
    modeStr.toUpperCase();

    uint8_t newMode = MODE_AUTO;
    bool ok = parse_mode_payload(modeStr.c_str(), &newMode);
    if (ok == false) {
        webServer.send(400, "text/plain", "invalid mode (AUTO/ON/OFF)");
        return;
    }

    if (target == "wiper") {
        apply_wiper_mode(newMode);
        webServer.send(200, "text/plain", "OK");
        return;
    }

    if (target == "high_beam" || target == "high") {
        apply_high_beam_mode(newMode);
        webServer.send(200, "text/plain", "OK");
        return;
    }

    webServer.send(400, "text/plain", "invalid target (wiper/high_beam)");
}

static void web_handle_turn()
{
    if (webServer.hasArg("mode") == false) {
        webServer.send(400, "text/plain", "missing mode");
        return;
    }

    String modeStr = webServer.arg("mode");
    modeStr.toUpperCase();

    uint8_t newMode = TURN_MODE_OFF;
    bool ok = parse_turn_mode_query(modeStr.c_str(), &newMode);
    if (ok == false) {
        webServer.send(400, "text/plain", "invalid mode (OFF/LEFT/RIGHT/HAZARD)");
        return;
    }

    uint8_t oldMode = g_turnMode;   // ★ 중요: 이전 상태 기억
    apply_turn_mode(newMode);

    // 1) hazard -> 다른 모드로 갈 때는 hazard부터 끔(안전)
    if (oldMode == TURN_MODE_HAZARD && newMode != TURN_MODE_HAZARD) {
        can_send_hazard_set(false);
    }

    // 2) 새 모드에 맞게 CONTROL이 이해하는 프레임 전송
    if (newMode == TURN_MODE_HAZARD) {
        can_send_hazard_set(true);
        g_lastSentTurnMode = g_turnMode;
        webServer.send(200, "text/plain", "OK");
        return;
    }

    if (newMode == TURN_MODE_LEFT) {
        can_send_turn_pulse(TURN_DIR_LEFT);   // 시작(또 누르면 CONTROL이 자체적으로 OFF 처리)
        g_lastSentTurnMode = g_turnMode;
        webServer.send(200, "text/plain", "OK");
        return;
    }

    if (newMode == TURN_MODE_RIGHT) {
        can_send_turn_pulse(TURN_DIR_RIGHT);
        g_lastSentTurnMode = g_turnMode;
        webServer.send(200, "text/plain", "OK");
        return;
    }

    // newMode == OFF
    // OFF는 "이전 모드"에 따라 끄는 방식으로 토글 1번 더 보내줌
    if (oldMode == TURN_MODE_LEFT) {
        can_send_turn_pulse(TURN_DIR_LEFT);   // 토글 OFF
    } else if (oldMode == TURN_MODE_RIGHT) {
        can_send_turn_pulse(TURN_DIR_RIGHT);  // 토글 OFF
    } else if (oldMode == TURN_MODE_HAZARD) {
        can_send_hazard_set(false);
    }

    g_lastSentTurnMode = g_turnMode;
    webServer.send(200, "text/plain", "OK");
}

static void web_setup_routes()
{
    webServer.on("/", HTTP_GET, web_handle_root);
    webServer.on("/api/state", HTTP_GET, web_handle_state);
    webServer.on("/api/set", HTTP_GET, web_handle_set);
    webServer.on("/api/turn", HTTP_GET, web_handle_turn);
    webServer.begin();
    Serial.println("[WEB] started on port 80");
}

/* =========================
 * AUTO 판단 / 명령 생성
 * ========================= */

static uint8_t compute_wiper_speed_from_humidity(float humidity)
{
    if (isnan(humidity)) {
        return kSpeedLevelStop;
    }

    if (humidity >= kWiperHumThFast) {
        return kSpeedLevelFast;
    }

    if (humidity >= kWiperHumThNormal) {
        return kSpeedLevelNormal;
    }

    if (humidity >= kWiperHumThSlow) {
        return kSpeedLevelSlow;
    }

    return kSpeedLevelStop;
}

static uint8_t compute_high_beam_state_from_cds(uint16_t cdsAdc, bool hasCds, uint8_t currentState)
{
    if (hasCds == false) {
        return 0;
    }

    uint8_t nextState = (currentState != 0U) ? 1U : 0U;

    if (kCdsDarkIsHighAdc) {
        if (cdsAdc >= kCdsOnThresholdAdc) {
            nextState = 1U;
        }

        if (cdsAdc <= kCdsOffThresholdAdc) {
            nextState = 0U;
        }

        return nextState;
    }

    /* bright -> high ADC, dark -> low ADC */
    if (cdsAdc <= kCdsOnThresholdAdc) {
        nextState = 1U;
    }

    if (cdsAdc >= kCdsOffThresholdAdc) {
        nextState = 0U;
    }

    return nextState;
}


static void controlLoopOnce()
{
    uint8_t desiredSpeedLevel = g_lastSentSpeedLevel;
    uint8_t desiredHighBeamState = g_lastSentHighBeamState;

    /* 센서 신선도 체크 */
    uint32_t nowMs = millis();

    bool dhtFresh = (g_lastSensorUpdateMs != 0U) && ((nowMs - g_lastSensorUpdateMs) <= kSensorStaleMs);
    bool cdsFresh = (g_lastCdsUpdateMs != 0U) && ((nowMs - g_lastCdsUpdateMs) <= kCdsStaleMs);

    float humidityForAuto = g_humidity;
    uint16_t cdsForAuto = g_cdsAdc;
    bool hasCdsForAuto = g_hasCds;

    if (dhtFresh == false) {
        humidityForAuto = NAN; /* AUTO에서는 STOP으로 떨어짐 */
    }

    if (cdsFresh == false) {
        hasCdsForAuto = false;
    }

    /* WIPER */
    if (g_wiperMode == MODE_AUTO) {
        desiredSpeedLevel = compute_wiper_speed_from_humidity(humidityForAuto);
    }
    else if (g_wiperMode == MODE_ON) {
        desiredSpeedLevel = kManualWiperOnSpeed;
    }
    else if (g_wiperMode == MODE_OFF) {
        desiredSpeedLevel = kSpeedLevelStop;
    }

    if (desiredSpeedLevel > kSpeedLevelFast) {
        desiredSpeedLevel = kSpeedLevelFast;
    }

    /* HIGH_BEAM */
    if (g_highBeamMode == MODE_AUTO) {
        uint8_t currentHighBeamState = g_lastSentHighBeamState;
        if (currentHighBeamState == 0xFF) {
            currentHighBeamState = 0U;
        }
        desiredHighBeamState = compute_high_beam_state_from_cds(cdsForAuto, hasCdsForAuto, currentHighBeamState);
    }
    else if (g_highBeamMode == MODE_ON) {
        desiredHighBeamState = 1U;
    }
    else if (g_highBeamMode == MODE_OFF) {
        desiredHighBeamState = 0U;
    }
    /* TURN (Right/Left/Emergency) */
    uint8_t desiredTurnMode = g_turnMode;
    if (desiredTurnMode > TURN_MODE_HAZARD) {
        desiredTurnMode = TURN_MODE_HAZARD;
    }



    /* 변경 시에만 송신 */
        if (g_lastSentSpeedLevel != desiredSpeedLevel) {
        bool ok = can_send_servo_speed(desiredSpeedLevel);
        if (ok) {
            g_lastSentSpeedLevel = desiredSpeedLevel;
            Serial.printf("[TX] SERVO speed_level=%u (H=%.1f T=%.1f)\n",
                (unsigned)g_lastSentSpeedLevel,
                (double)g_humidity,
                (double)g_temperature);
        }
    }

        if (g_lastSentHighBeamState != desiredHighBeamState) {
        bool ok = can_send_high_beam_state(desiredHighBeamState);
        if (ok) {
            g_lastSentHighBeamState = desiredHighBeamState;
            Serial.printf("[TX] HIGH_BEAM state=%u (CDS=%u)\n",
                (unsigned)g_lastSentHighBeamState,
                (unsigned)g_cdsAdc);
              }
        }


  if (g_lastSentTurnMode != desiredTurnMode) {
    uint8_t prevMode = g_lastSentTurnMode;

    bool ok = send_turn_transition(prevMode, desiredTurnMode);
    if (ok) {
        g_lastSentTurnMode = desiredTurnMode;
        Serial.printf("[TX] TURN mode=%s\n", turn_mode_to_string(g_lastSentTurnMode));
    }
  }
}

void setup()
{
    Serial.begin(115200);
    delay(300);
    Serial.println("\nBOOT (MQTT gateway node)");

    if (canInit() == false) {
        Serial.println("CAN init failed");
        while (true) { delay(1000); }
    }

    build_device_id_from_mac();
    build_topics();

    wifi_connect();
    mqtt_connect();

    web_setup_routes();

    /* 부팅 직후 1회 강제 송신 */
    g_lastSentSpeedLevel = 0xFF;
    g_lastSentHighBeamState = 0xFF;
    g_lastSentTurnMode = 0xFF;
    controlLoopOnce();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED) {
      wifi_connect();
  }

  if (mqttClient.connected() == false) {
      mqtt_connect();
  }

  mqttClient.loop();
  webServer.handleClient();

  canPollSensors();

  /* 100ms마다 AUTO/수동 판단 + 필요한 명령 송신 */
  static uint32_t lastControlMs = 0;
  uint32_t nowMs = millis();
  if ((nowMs - lastControlMs) >= 100) {
      lastControlMs = nowMs;
      controlLoopOnce();
  }

  /* 유실 대비 2초마다 현재 명령 재송신(선택) */
  static uint32_t lastResendMs = 0;
  if ((nowMs - lastResendMs) >= 2000) {
    lastResendMs = nowMs;

    if (g_lastSentSpeedLevel != 0xFF) {
      if (can_send_servo_speed(g_lastSentSpeedLevel) == false) {
      Serial.println("[CAN-TX-FAIL] resend SERVO");
      }
    }

    if (g_lastSentHighBeamState != 0xFF) {
      if (can_send_high_beam_state(g_lastSentHighBeamState) == false) {
      Serial.println("[CAN-TX-FAIL] resend HIGH_BEAM");
      }
    }
  }
  
  delay(5);
}
