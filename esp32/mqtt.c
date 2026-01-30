#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <WebServer.h>

/* =========================
 * WiFi / MQTT 설정
 * ========================= */
static const char* WIFI_SSID = "e^(ix) = k cosx + ki sinx, k = ?";
static const char* WIFI_PASS = "haha5123";

static const char* MQTT_HOST = "broker.emqx.io";
static const uint16_t MQTT_PORT = 1883;

/* =========================
 * SPI 배선(프로젝트에 맞게 수정)
 * ========================= */
static const int PIN_SPI_CS = 5;
static const int PIN_SPI_SCK = 18;
static const int PIN_SPI_MISO = 19;
static const int PIN_SPI_MOSI = 23;

/* =========================
 * SPI 프레임(STM32가 이대로 파싱해야 함)
 * [0]=SOF 0xA5
 * [1]=VER 0x01
 * [2]=WIPER_MODE (0=AUTO,1=ON,2=OFF, 0xFF=NOCHANGE)
 * [3]=HIGH_MODE  (0=AUTO,1=ON,2=OFF, 0xFF=NOCHANGE)
 * [4]=CRC XOR([0..3])
 * [5]=EOF 0x5A
 * ========================= */
static const uint8_t SPI_FRAME_SOF = 0xA5;
static const uint8_t SPI_FRAME_VER = 0x01;
static const uint8_t SPI_FRAME_EOF = 0x5A;

static const uint8_t SPI_MODE_NOCHANGE = 0xFF;

static const uint8_t MODE_AUTO = 0;
static const uint8_t MODE_ON = 1;
static const uint8_t MODE_OFF = 2;

/* 토픽 베이스 */
static const char* TOPIC_BASE = "Lim/haha5123/esp32";

/* =========================
 * 전역 상태
 * ========================= */
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer webServer(80);

static uint8_t g_wiperMode = MODE_AUTO;
static uint8_t g_highMode = MODE_AUTO;

static char g_deviceId[16];

static char g_topicWiperCmd[96];
static char g_topicHighCmd[96];
static char g_topicWiperState[96];
static char g_topicHighState[96];
static char g_topicOnline[96];

/* =========================
 * 유틸
 * ========================= */
static uint8_t calc_crc_xor_4(const uint8_t* bytes4)
{
    uint8_t crc = 0;
    crc ^= bytes4[0];
    crc ^= bytes4[1];
    crc ^= bytes4[2];
    crc ^= bytes4[3];
    return crc;
}

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
 * SPI 송신
 * ========================= */
static void spi_send_modes(uint8_t wiperModeOrNoChange, uint8_t highModeOrNoChange)
{
    uint8_t tx[6];
    uint8_t rx[6];

    tx[0] = SPI_FRAME_SOF;
    tx[1] = SPI_FRAME_VER;
    tx[2] = wiperModeOrNoChange;
    tx[3] = highModeOrNoChange;
    tx[4] = calc_crc_xor_4(tx);
    tx[5] = SPI_FRAME_EOF;

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS, LOW);
    delayMicroseconds(2);
    SPI.transferBytes(tx, rx, 6);
    delayMicroseconds(2);
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
}

/* =========================
 * MQTT publish
 * ========================= */
static void mqtt_publish_state()
{
    mqttClient.publish(g_topicWiperState, mode_to_string(g_wiperMode), true);
    mqttClient.publish(g_topicHighState, mode_to_string(g_highMode), true);
}

/* =========================
 * 모드 적용(중복 제거)
 * ========================= */
static void apply_wiper_mode(uint8_t newMode)
{
    if (g_wiperMode == newMode) {
        return;
    }

    g_wiperMode = newMode;
    Serial.printf("[APPLY] WIPER=%s\n", mode_to_string(g_wiperMode));

    spi_send_modes(g_wiperMode, SPI_MODE_NOCHANGE);
    mqtt_publish_state();
}

static void apply_high_mode(uint8_t newMode)
{
    if (g_highMode == newMode) {
        return;
    }

    g_highMode = newMode;
    Serial.printf("[APPLY] HIGH=%s\n", mode_to_string(g_highMode));

    spi_send_modes(SPI_MODE_NOCHANGE, g_highMode);
    mqtt_publish_state();
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

    if (strcmp(topic, g_topicHighCmd) == 0) {
        Serial.printf("[MQTT] HIGH=%s\n", mode_to_string(newMode));
        apply_high_mode(newMode);
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
    snprintf(g_topicHighCmd, sizeof(g_topicHighCmd), "%s/high/cmd/%s", TOPIC_BASE, g_deviceId);

    snprintf(g_topicWiperState, sizeof(g_topicWiperState), "%s/wiper/state/%s", TOPIC_BASE, g_deviceId);
    snprintf(g_topicHighState, sizeof(g_topicHighState), "%s/high/state/%s", TOPIC_BASE, g_deviceId);

    snprintf(g_topicOnline, sizeof(g_topicOnline), "%s/online/%s", TOPIC_BASE, g_deviceId);

    Serial.println("[TOPICS]");
    Serial.println(g_topicWiperCmd);
    Serial.println(g_topicHighCmd);
    Serial.println(g_topicWiperState);
    Serial.println(g_topicHighState);
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
    mqttClient.subscribe(g_topicHighCmd);

    mqttClient.publish(g_topicOnline, "online", true);
    mqtt_publish_state();
}

/* =========================
 * 정환님 기존 HTML (JS만 최소 수정)
 *  - 버튼 누르면 /api/set 로 명령 전송
 *  - 1초마다 /api/state 로 현재 상태 동기화
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
      <div style="font-size:22px; margin-bottom:10px; color: rgba(255,255,255,.85);">가로 모드에서 사용하십시오.</div>
      <div style="font-size:14px; color: rgba(255,255,255,.55);">태블릿을 가로로 눕히면 컨트롤 패널이 표시됩니다.</div>
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
            <div class="iconLabel">Emer</div>
          </div>
        </button>
      </div>

      <div class="controlGroup" data-group="high">
        <div class="iconCard">
          <div class="iconInner">
            <svg class="svgIcon" viewBox="0 0 64 64" aria-hidden="true">
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                d="M18 36c6-10 22-10 28 0"/>
              <path class="strokeNeon" fill="none" stroke-width="4" stroke-linecap="round"
                d="M20 28h-6M22 24h-8M22 32h-8"/>
              <text x="36" y="40" fill="rgba(102,243,220,.85)" font-size="10" font-weight="700">AUTO</text>
              <text x="38" y="52" fill="rgba(102,243,220,.85)" font-size="12" font-weight="800">High</text>
            </svg>
            <div class="iconLabel">High</div>
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
          <div>Temperature: <b id="tempText">--°C</b></div>
          <div>Humidity: <b id="humText">--%</b></div>
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

      // wiper/high 상태 반영
      const wiperGroup = document.querySelector('.controlGroup[data-group="wiper"]');
      const highGroup  = document.querySelector('.controlGroup[data-group="high"]');

      if (wiperGroup) {
        const state = (j.wiper || 'AUTO').toLowerCase(); // 'auto'/'on'/'off'
        setGroupUI(wiperGroup, state);
      }

      if (highGroup) {
        const state = (j.high || 'AUTO').toLowerCase();
        setGroupUI(highGroup, state);
      }

      // (선택) 온습도 표시: 지금은 서버가 값 안 주면 -- 유지
      if (typeof j.temp === 'number') {
        document.getElementById('tempText').textContent = j.temp.toFixed(1) + '°C';
      }
      if (typeof j.hum === 'number') {
        document.getElementById('humText').textContent = j.hum.toFixed(1) + '%';
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
      if (groupName === 'wiper' || groupName === 'high') {
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
    leftButton.addEventListener('click', () => {
      if (isHazardOn()) return;
      const wantOn = !leftButton.classList.contains('active');
      setTurnSignal(wantOn ? 'left' : 'off');
    });
  }

  if (rightButton) {
    rightButton.addEventListener('click', () => {
      if (isHazardOn()) return;
      const wantOn = !rightButton.classList.contains('active');
      setTurnSignal(wantOn ? 'right' : 'off');
    });
  }

  if (emerButton) {
    emerButton.addEventListener('click', () => {
      const nextOn = !emerButton.classList.contains('active');
      setHazard(nextOn);
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

/* =========================
 * Web API
 * ========================= */
static void web_handle_root()
{
    // 큰 HTML은 send_P로 전송해야 RAM이 안전합니다.
    webServer.send_P(200, "text/html", INDEX_HTML);
}

static void web_handle_state()
{
    String json = "{";
    json += "\"deviceId\":\""; json += g_deviceId; json += "\",";
    json += "\"topicBase\":\""; json += TOPIC_BASE; json += "\",";
    json += "\"wiper\":\""; json += mode_to_string(g_wiperMode); json += "\",";
    json += "\"high\":\"";  json += mode_to_string(g_highMode);  json += "\"";
    // temp/hum은 현재 mqtt 노드가 보유하는 값이 없으므로 일부러 안 넣었습니다.
    // 필요하면 추후 json += ",\"temp\":...,\"hum\":..." 형태로 확장하면 됩니다.
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

    if (target == "high") {
        apply_high_mode(newMode);
        webServer.send(200, "text/plain", "OK");
        return;
    }

    webServer.send(400, "text/plain", "invalid target (wiper/high)");
}

static void web_setup_routes()
{
    webServer.on("/", HTTP_GET, web_handle_root);
    webServer.on("/api/state", HTTP_GET, web_handle_state);
    webServer.on("/api/set", HTTP_GET, web_handle_set);
    webServer.begin();
    Serial.println("[WEB] started on port 80");
}

/* =========================
 * setup / loop
 * ========================= */
void setup()
{
    Serial.begin(115200);
    delay(300);

    pinMode(PIN_SPI_CS, OUTPUT);
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI, PIN_SPI_CS);

    build_device_id_from_mac();
    build_topics();

    wifi_connect();
    mqtt_connect();

    web_setup_routes();

    // 부팅 시 1회 동기화
    spi_send_modes(g_wiperMode, g_highMode);
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

    // 유실 대비 주기적 SPI 동기화(선택)
    static uint32_t lastSyncMs = 0;
    uint32_t nowMs = millis();
    if ((nowMs - lastSyncMs) >= 2000) {
        lastSyncMs = nowMs;
        spi_send_modes(g_wiperMode, g_highMode);
    }

    delay(5);
}
