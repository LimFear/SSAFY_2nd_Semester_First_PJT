#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <math.h>

#include "driver/twai.h"

/* =========================
 * WiFi / MQTT 설정
 * ========================= */
static const char* WIFI_SSID = "e^(ix) = k cosx + ki sinx, k = ?";
static const char* WIFI_PASS = "haha5123";

static const char* MQTT_HOST = "broker.emqx.io";
static const uint16_t MQTT_PORT = 1883;

/* 토픽 베이스 */
static const char* TOPIC_BASE = "Lim/haha5123/esp32";

/* =========================
 * CAN 핀/ID
 * ========================= */
#define CAN_RX_GPIO_PIN GPIO_NUM_32
#define CAN_TX_GPIO_PIN GPIO_NUM_33

#define CAN_DHT_ID              0x100
#define CAN_CDS_ID              0x110

#define CAN_ID_CMD_SERVO        0x200
#define SERVO_CMD_SET_ANGLE     0x01

#define CAN_ID_CMD_HIGH_BEAM    0x210
#define HIGH_BEAM_CMD_SET_STATE 0x11


#define CAN_ID_CMD_TURN           0x220
#define TURN_CMD_PULSE            0x31
#define TURN_CMD_HAZARD_SET       0x32
#define TURN_DIR_LEFT             0x00
#define TURN_DIR_RIGHT            0x01


 /* =========================
  * 모드 상수
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
      --padTopH: clamp(66px, 12vh, 120px);
      --padBtnH:  clamp(70px, 13vh, 132px);
      --gap: clamp(8px, 1.2vh, 14px);
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

    .app{
      height:100vh;
      width:100vw;
      padding: clamp(8px, 1.4vh, 14px) clamp(10px, 1.8vw, 18px);
      display:grid;
      grid-template-rows: 30vh 22vh 48vh;
      gap: var(--gap);
    }

    .topRow{
      display:grid;
      grid-template-columns: repeat(5, 1fr);
      gap: clamp(8px, 1.2vw, 14px);
      align-items:stretch;
      min-height:0;
    }

    .controlGroup{
      display:grid;
      grid-template-rows: 1fr auto;
      gap: clamp(6px, 0.9vh, 10px);
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
      padding: clamp(8px, 1vh, 10px);
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
      gap: clamp(6px, .8vh, 10px);
    }

    .iconLabel{
      font-size: clamp(12px, 1.6vh, 18px);
      color: rgba(255,255,255,.65);
      letter-spacing:.3px;
      padding-bottom:2px;
    }

    .svgIcon{
      width: clamp(56px, 6.2vw, 108px);
      height:auto;
      max-height:70%;
      opacity:.95;
      filter: drop-shadow(0 0 8px rgba(102,243,220,.18));
    }
    .strokeNeon{ stroke:var(--neon); }
    .fillNeon{ fill:var(--neon); }

    .switchStack{
      display:grid;
      grid-template-rows: auto auto;
      gap: clamp(6px, 0.8vh, 10px);
    }

    .segSwitch{
      width:100%;
      display:grid;
      grid-template-columns: 1fr 1fr;
      gap: clamp(6px, .8vw, 12px);
    }

    .segBtn{
      height: clamp(30px, 5vh, 42px);
      border-radius: 11px;
      border: 2px solid var(--neon-dim);
      background: transparent;
      color: rgba(255,255,255,.70);
      font-size: clamp(13px, 1.7vh, 18px);
      user-select:none;
      -webkit-tap-highlight-color:transparent;
      touch-action: manipulation;
    }
    .segBtn.active{
      border-color: rgba(102,243,220,.95);
      box-shadow: var(--shadow-strong);
      background: rgba(102,243,220,.10);
      color: rgba(255,255,255,.92);
    }
    .segBtn:active{ transform: translateY(1px); }

    .autoBtn{
      height: clamp(28px, 4.6vh, 40px);
      border-radius: 11px;
      border: 2px solid var(--neon-dim);
      background: transparent;
      color: rgba(255,255,255,.68);
      font-size: clamp(12px, 1.6vh, 17px);
      letter-spacing: .6px;
      user-select:none;
      -webkit-tap-highlight-color:transparent;
      touch-action: manipulation;
    }
    .autoBtn.active{
      border-color: rgba(102,243,220,.95);
      box-shadow: var(--shadow-strong);
      background: rgba(102,243,220,.10);
      color: rgba(255,255,255,.92);
    }
    .autoBtn:active{ transform: translateY(1px); }

    .midRow{
      display:grid;
      grid-template-columns: 1fr;
      align-items: start;
      min-height:0;
    }
    .infoBlock{
      display:grid;
      grid-template-columns: auto 1fr;
      gap: clamp(12px, 1.6vw, 18px);
      align-items: start;
      min-height:0;
    }

    .readouts{
      font-size: clamp(14px, 2.2vh, 24px);
      line-height: 1.45;
      color: var(--text-dim);
      letter-spacing: .2px;
      padding-top: 2px;
    }
    .readouts b{
      color: rgba(255,255,255,.82);
      font-weight: 650;
    }

    .resetBtn{
      width: clamp(130px, 18vw, 180px);
      height: clamp(86px, 15vh, 140px);
      border-radius: var(--radius);
      border: 2px solid var(--neon-dim);
      background: transparent;
      box-shadow: var(--shadow);
      color: rgba(255,255,255,.70);
      font-size: clamp(14px, 1.8vh, 20px);
      line-height: 1.2;
      user-select:none;
      -webkit-tap-highlight-color:transparent;
      touch-action: manipulation;
    }
    .resetBtn:active{ box-shadow: var(--shadow-strong); transform: translateY(1px); }

    .bottomRow{
      display:grid;
      grid-template-columns: 1fr clamp(170px, 22vw, 260px);
      gap: clamp(12px, 2vw, 22px);
      align-items: center;
      min-height:0;
    }

    .pad{
      width: min(640px, 100%);
      display:grid;
      grid-template-rows: auto auto;
      gap: clamp(10px, 1.4vh, 14px);
      justify-items: center;
      align-items: start;
      min-height:0;
      padding-top: 2px;
    }

    .padTop{
      width: clamp(170px, 22vw, 220px);
    }

    .padBottom{
      width: 100%;
      display:grid;
      grid-template-columns: 1fr 1fr 1fr;
      gap: clamp(10px, 1.4vw, 14px);
    }

    .padBtn{
      border-radius: var(--radius);
      border: 2px solid var(--neon-dim);
      background: transparent;
      box-shadow: var(--shadow);
      display:grid;
      place-items:center;
      user-select:none;
      -webkit-tap-highlight-color:transparent;
      touch-action: none;
      position: relative;
    }
    .padBtn.top{ height: var(--padTopH); }
    .padBtn.side{ height: var(--padBtnH); }

    .padBtn .corner{
      position:absolute;
      width: clamp(14px, 2vw, 24px);
      height: clamp(14px, 2vw, 24px);
      border-color: rgba(102,243,220,.45);
      border-style: solid;
    }
    .padBtn .c1{ top: 10px; left: 10px; border-width: 3px 0 0 3px; }
    .padBtn .c2{ top: 10px; right: 10px; border-width: 3px 3px 0 0; }
    .padBtn .c3{ bottom: 10px; left: 10px; border-width: 0 0 3px 3px; }
    .padBtn .c4{ bottom: 10px; right: 10px; border-width: 0 3px 3px 0; }

    .padBtn svg{
      width: 58%;
      height: 58%;
      opacity: .92;
      filter: drop-shadow(0 0 10px rgba(102,243,220,.18));
    }

    .padBtn.pressed{
      box-shadow: var(--shadow-strong);
      border-color: rgba(102,243,220,.95);
      background: rgba(102,243,220,.08);
    }

    .stopBtn{
      width: 100%;
      aspect-ratio: 1 / 1;
      border-radius: var(--radius);
      border: 2px solid var(--neon-dim);
      background: transparent;
      box-shadow: var(--shadow);
      position: relative;
      user-select:none;
      -webkit-tap-highlight-color:transparent;
      touch-action: manipulation;
      overflow: hidden;
    }
    .stopRing{
      position:absolute;
      width:72%;
      height:72%;
      border: clamp(3px, .6vw, 6px) solid rgba(102,243,220,.55);
      border-radius: 50%;
      left: 14%;
      top: 14%;
      transform: rotate(18deg);
      box-shadow: 0 0 16px rgba(102,243,220,.12);
    }
    .stopStamp{
      position:absolute;
      inset:0;
      display:grid;
      place-items:center;
      transform: rotate(-18deg);
      color: rgba(102,243,220,.88);
      font-weight: 800;
      font-size: clamp(28px, 4.6vw, 54px);
      letter-spacing: 2px;
      text-shadow: 0 0 14px rgba(102,243,220,.20);
    }
    .stopBtn:active{ box-shadow: var(--shadow-strong); transform: translateY(1px); }
  </style>
</head>

<body>
  <div class="portraitLock">
    <div>
      <div style="font-size:22px; margin-bottom:10px; color: rgba(255,255,255,.85);">���� ��忡�� ����Ͻʽÿ�.</div>
      <div style="font-size:14px; color: rgba(255,255,255,.55);">�º����� ���η� ������ ��Ʈ�� �г��� ǥ�õ˴ϴ�.</div>
    </div>
  </div>

  <div class="app">

    <div class="topRow">

      <div class="controlGroup simple" data-group="left">
        <button class="iconCard iconCardBtn" type="button" data-box="left" aria-label="Left">
          <div class="iconInner">
            <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
              <path class="fillNeon" d="M28 12 8 32l20 20v-12h28V24H28V12z"/>
            </svg>
            <div class="iconLabel">Left</div>
          </div>
        </button>
      </div>

      <div class="controlGroup" data-group="wiper">
        <div class="iconCard">
          <div class="iconInner">
            <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                d="M10 28c10-10 34-10 44 0M18 36c8-7 20-7 28 0"/>
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round" d="M32 18v28"/>
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round" d="M27 40l10-8"/>
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

      <div class="controlGroup simple" data-group="emer">
        <button class="iconCard iconCardBtn" type="button" data-box="emer" aria-label="Emergency">
          <div class="iconInner">
            <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
              <path class="fillNeon" d="M22 30c0-6 4-10 10-10s10 4 10 10v4H22v-4z"/>
              <path class="fillNeon" d="M18 38h28l4 12H14l4-12z"/>
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round" d="M32 10v6"/>
            </svg>
            <div class="iconLabel">Emergency</div>
          </div>
        </button>
      </div>

      <div class="controlGroup" data-group="high_beam">
        <div class="iconCard">
          <div class="iconInner">
            <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                d="M18 36c6-10 22-10 28 0"/>
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                d="M20 28h-6M22 24h-8M22 32h-8"/>
              <text x="36" y="40" fill="rgba(102,243,220,.85)" font-size="10" font-weight="700">AUTO</text>
              <text x="38" y="52" fill="rgba(102,243,220,.85)" font-size="12" font-weight="800">High Beam</text>
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

      <div class="controlGroup simple" data-group="right">
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

    <div class="midRow">
      <div class="infoBlock">
        <button class="resetBtn" type="button">Reset<br/>Rotation</button>

        <div class="readouts">
          <div>Temperature: <b id="tempText">--C</b></div>
          <div>Humidity: <b id="humText">--%</b></div>
          <div>CDS(ADC): <b id="cdsText">--</b></div>
        </div>
      </div>
    </div>

    <div class="bottomRow">

      <div class="pad" aria-label="Arrow Pad">
        <div class="padTop">
          <div class="padBtn top" data-pad="up" role="button" aria-label="Up">
            <div class="corner c1"></div><div class="corner c2"></div><div class="corner c3"></div><div class="corner c4"></div>
            <svg viewBox="0 0 64 64" aria-hidden="true">
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M32 14 20 26m12-12 12 12"/>
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M32 26 20 38m12-12 12 12"/>
            </svg>
          </div>
        </div>

        <div class="padBottom">
          <div class="padBtn side" data-pad="left" role="button" aria-label="Left">
            <div class="corner c1"></div><div class="corner c2"></div><div class="corner c3"></div><div class="corner c4"></div>
            <svg viewBox="0 0 64 64" aria-hidden="true">
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M14 32 26 20m-12 12 12 12"/>
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M26 32 38 20m-12 12 12 12"/>
            </svg>
          </div>

          <div class="padBtn side" data-pad="down" role="button" aria-label="Down">
            <div class="corner c1"></div><div class="corner c2"></div><div class="corner c3"></div><div class="corner c4"></div>
            <svg viewBox="0 0 64 64" aria-hidden="true">
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M32 50 20 38m12 12 12-12"/>
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M32 38 20 26m12 12 12-12"/>
            </svg>
          </div>

          <div class="padBtn side" data-pad="right" role="button" aria-label="Right">
            <div class="corner c1"></div><div class="corner c2"></div><div class="corner c3"></div><div class="corner c4"></div>
            <svg viewBox="0 0 64 64" aria-hidden="true">
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M50 32 38 20m12 12-12 12"/>
              <path class="strokeNeon" fill="none" stroke-width="5" stroke-linecap="round" stroke-linejoin="round"
                d="M38 32 26 20m12 12-12 12"/>
            </svg>
          </div>
        </div>
      </div>

      <button class="stopBtn" type="button" aria-label="STOP">
        <div class="stopRing" aria-hidden="true"></div>
        <div class="stopStamp">STOP</div>
      </button>

    </div>

  </div>

<script>
  // ===== 공용 유틸 =====
  function setActive(element, on) {
    element.classList.toggle('active', Boolean(on));
  }

  // ===== [추가] ESP32로 명령 보내기 =====
  async function sendMode(target, state) {
    // state: 'auto' | 'on' | 'off'
    let mode = 'AUTO';
    if (state === 'on') mode = 'ON';
    if (state === 'off') mode = 'OFF';

    try {
      await fetch('/api/set?target=' + encodeURIComponent(target) + '&mode=' + encodeURIComponent(mode));
    } catch (e) {
      // 네트워크 끊겨도 UI가 멈추지 않게 무시
    }
  }

  // ===== [추가] 서버 상태 동기화 =====
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

  async function refreshFromServer(){
    try{
      const r = await fetch('/api/state');
      const j = await r.json();

      // wiper/high_beam 상태 반영
      const wiperGroup = document.querySelector('.controlGroup[data-group="wiper"]');
      const highGroup  = document.querySelector('.controlGroup[data-group="high_beam"]');

      if (wiperGroup) {
        const state = (j.wiper || 'AUTO').toLowerCase(); // 'auto'/'on'/'off'
        setGroupUI(wiperGroup, state);
      }

      if (highGroup) {
        const state = ((j.high_beam || j.high || 'AUTO')).toLowerCase();
        setGroupUI(highGroup, state);
      }

      // turn 상태 반영 (OFF/LEFT/RIGHT/HAZARD)
      if (typeof j.turn === 'string') {
        const t = (j.turn || 'OFF').toLowerCase();

        if (t === 'hazard') {
          setHazard(true);
        } else {
          setHazard(false);

          if (t === 'left') {
            setTurnSignal('left');
          } else if (t === 'right') {
            setTurnSignal('right');
          } else {
            setTurnSignal('off');
          }
        }
      }

      // (선택) 온습도 표시: 지금은 서버가 값 안 주면 -- 유지
      if (typeof j.temp === 'number') {
        document.getElementById('tempText').textContent = j.temp.toFixed(1) + '��C';
      }
      if (typeof j.hum === 'number') {
        document.getElementById('humText').textContent = j.hum.toFixed(1) + '%';
      }
      if (typeof j.cds === 'number') {
        document.getElementById('cdsText').textContent = String(j.cds);
      }
    } catch(e) {
      // 무시
    }
  }

  // ===== 1) AUTO 기본값 + [추가] 클릭 시 ESP32로 전송 =====
  document.querySelectorAll('.controlGroup').forEach(group => {
    const groupName = group.dataset.group || '';
    const segmentButtons = group.querySelectorAll('.segBtn');
    const autoButton = group.querySelector('.autoBtn');

    if (!autoButton) {
      return;
    }

    function clearAll() {
      segmentButtons.forEach(button => button.classList.remove('active'));
      autoButton.classList.remove('active');
    }

    function setState(state) {
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

      // [추가] wiper/high면 서버로 명령 전송
      if (groupName === 'wiper' || groupName === 'high_beam') {
        sendMode(groupName, state);
      }
    }

    segmentButtons.forEach(button => {
      button.addEventListener('click', () => setState(button.dataset.seg));
    });

    autoButton.addEventListener('click', () => setState('auto'));

    setState('auto');
  });

  // ===== 버튼 참조 =====
  const leftButton  = document.querySelector('[data-box="left"]');
  const rightButton = document.querySelector('[data-box="right"]');
  const emerButton  = document.querySelector('[data-box="emer"]');


  // ===== [추가] ESP32로 방향지시등/비상등 명령 보내기 =====
  async function sendTurn(mode) {
    // mode: 'OFF' | 'LEFT' | 'RIGHT' | 'HAZARD'
    try {
      await fetch('/api/turn?mode=' + encodeURIComponent(mode));
    } catch (e) {
      // 무시
    }
  }

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
    if (!emerButton) return;

    setActive(emerButton, on);
    if (leftButton)  setActive(leftButton, on);
    if (rightButton) setActive(rightButton, on);
  }

  if (leftButton) {
    leftButton.addEventListener('click', async () => {
      if (isHazardOn()) {
        setHazard(false);
        await sendTurn('OFF');
      }

      const wantOn = !leftButton.classList.contains('active');
      setTurnSignal(wantOn ? 'left' : 'off');
      await sendTurn(wantOn ? 'LEFT' : 'OFF');
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
    });
  }

  if (emerButton) {
    emerButton.addEventListener('click', async () => {
      const nextOn = !emerButton.classList.contains('active');
      setHazard(nextOn);
      await sendTurn(nextOn ? 'HAZARD' : 'OFF');
    });
  }

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

  document.querySelectorAll('.padBtn').forEach(button => {
    button.addEventListener('pointerdown', (event) => {
      event.preventDefault();
      button.setPointerCapture(event.pointerId);
      button.classList.add('pressed');
    });

    ['pointerup', 'pointercancel', 'pointerleave'].forEach(eventName => {
      button.addEventListener(eventName, () => button.classList.remove('pressed'));
    });
  });

  // [추가] 1초마다 서버 상태 동기화
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
    tx.data[0] = SERVO_CMD_SET_ANGLE;
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

        if ((rxMessage.flags & TWAI_MSG_FLAG_EXTD) != 0) {
            continue;
        }

        if (rxMessage.identifier == CAN_DHT_ID)
        {
            if (rxMessage.data_length_code < 4) {
                continue;
            }

            int16_t humidity_x10 = (int16_t)((uint16_t)rxMessage.data[0] | ((uint16_t)rxMessage.data[1] << 8));
            int16_t temp_x10 = (int16_t)((uint16_t)rxMessage.data[2] | ((uint16_t)rxMessage.data[3] << 8));

            g_humidity = ((float)humidity_x10) / 10.0f;
            g_temperature = ((float)temp_x10) / 10.0f;
            g_lastSensorUpdateMs = millis();
            continue;
        }

        if (rxMessage.identifier == CAN_CDS_ID)
        {
            if (rxMessage.data_length_code < 2) {
                continue;
            }

            uint16_t cdsAdc12 = (uint16_t)((uint16_t)rxMessage.data[0] | ((uint16_t)rxMessage.data[1] << 8));
            g_cdsAdc = cdsAdc12;
            g_hasCds = true;
            g_lastCdsUpdateMs = millis();
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
 * 토픽 생성
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
    mqttClient.setCallback(mqtt_callback);

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

    mqttClient.subscribe(g_topicWiperCmd);
    mqttClient.subscribe(g_topicHighBeamCmd);
    mqttClient.subscribe(g_topicHighCmdCompat);

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
    json += "\"";


    json += ",\"turn\":\"";
    json += turn_mode_to_string(g_turnMode);
    json += "\"";
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
    }
    else if (oldMode == TURN_MODE_RIGHT) {
        can_send_turn_pulse(TURN_DIR_RIGHT);  // 토글 OFF
    }
    else if (oldMode == TURN_MODE_HAZARD) {
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

    /* WIPER */
    if (g_wiperMode == MODE_AUTO) {
        desiredSpeedLevel = compute_wiper_speed_from_humidity(g_humidity);
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
        desiredHighBeamState = compute_high_beam_state_from_cds(g_cdsAdc, g_hasCds, g_lastSentHighBeamState);
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