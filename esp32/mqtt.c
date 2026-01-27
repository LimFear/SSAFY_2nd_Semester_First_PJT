#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>

/* ===== Wi-Fi ===== */
const char* ssid = "e^(ix) = k cosx + ki sinx, k = ?"; //개인 와이파이 이름 입력
const char* password = "haha0223"; //개인 와이파이 비밀번호 입력

/* ===== MQTT Broker (EMQX Public) =====
   Host: broker.emqx.io
   TCP: 1883
   WS:  8083
   WSS: 8084  (browser uses this)  path: /mqtt
*/
const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;

/* ===== Topics ===== */
const char* TOPIC_CMD = "Lim/esp32/led27/cmd/haha5123";
const char* TOPIC_STATE = "Lim/esp32/led27/state/haha5123";

/* ===== GPIO ===== */
constexpr uint8_t LED_GPIO = 27;

/* ===== Network Clients ===== */
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
WebServer webServer(80);

/* ===== Web Page (served by ESP32) ===== */
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

      /* 하단 패드 높이(기기마다 자동 스케일) */
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

    /* ===== 전체 레이아웃: 하단 영역(화살표+STOP) 확실히 확보 ===== */
    .app{
      height:100vh;
      width:100vw;
      padding: clamp(8px, 1.4vh, 14px) clamp(10px, 1.8vw, 18px);
      display:grid;
      grid-template-rows: 30vh 22vh 48vh; /* TOP / MID / BOTTOM */
      gap: var(--gap);
    }

    /* ===== TOP: 5개 컨트롤 ===== */
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
      grid-template-rows: 1fr; /* Left/Right: 스위치 없음 */
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

    /* Left/Right는 박스 자체가 버튼 */
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

    /* ===== ON/OFF + AUTO ===== */
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

    /* ===== MID: 온습도 + Reset ===== */
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

    /* ===== BOTTOM: 화살표 + STOP (옆 배치) ===== */
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
      align-items: start; /* 위로 올려서 잘림 방지 */
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

    /* STOP (하단 오른쪽) */
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

    <!-- ===== TOP ===== -->
    <div class="topRow">

      <!-- Left (박스 자체 버튼) -->
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

      <!-- Wiper (ON/OFF + AUTO) -->
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

      <!-- Emer (ON/OFF + AUTO) -->
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


      <!-- High (ON/OFF + AUTO) -->
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

      <!-- Right (박스 자체 버튼) -->
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

    <!-- ===== MID ===== -->
    <div class="midRow">
      <div class="infoBlock">
        <button class="resetBtn" type="button">Reset<br/>Rotation</button>

        <div class="readouts">
          <div>Temperature: <b>27°C</b></div>
          <div>Humidity: <b>49%</b></div>
        </div>
      </div>
    </div>

    <!-- ===== BOTTOM: 화살표 + STOP ===== -->
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

  // ===== 1) AUTO 기본값 =====
  document.querySelectorAll('.controlGroup').forEach(group => {
    const segmentButtons = group.querySelectorAll('.segBtn');
    const autoButton = group.querySelector('.autoBtn');

    if (!autoButton) {
      return; // AUTO가 없는 그룹(Left/Right/Emer)은 스킵
    }

    function clearAll() {
      segmentButtons.forEach(button => button.classList.remove('active'));
      autoButton.classList.remove('active');
    }

    function setState(state) {
      clearAll();

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

    segmentButtons.forEach(button => {
      button.addEventListener('click', () => setState(button.dataset.seg));
    });

    autoButton.addEventListener('click', () => setState('auto'));

    // 기본값: AUTO
    setState('auto');
  });

  // ===== 버튼 참조 =====
  const leftButton  = document.querySelector('[data-box="left"]');
  const rightButton = document.querySelector('[data-box="right"]');
  const emerButton  = document.querySelector('[data-box="emer"]');

  function isHazardOn() {
    if (!emerButton) {
      return false;
    }
    return emerButton.classList.contains('active');
  }

  // ===== 2) Left/Right 상호배타 =====
  function setTurnSignal(side) { // 'left' | 'right' | 'off'
    if (!leftButton || !rightButton) {
      return;
    }

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

    // off
    setActive(leftButton, false);
    setActive(rightButton, false);
  }

  // ===== 3) Emergency(비상등): ON이면 Left+Right 둘 다 ON =====
  function setHazard(on) {
    if (!emerButton) {
      return;
    }

    setActive(emerButton, on);

    if (leftButton) {
      setActive(leftButton, on);
    }
    if (rightButton) {
      setActive(rightButton, on);
    }
  }

  // ===== 정책 B: 비상등 ON 동안 Left/Right 클릭 무시 =====
  if (leftButton) {
    leftButton.addEventListener('click', () => {
      if (isHazardOn()) {
        return; // 무시(비상등 유지)
      }

      const wantOn = !leftButton.classList.contains('active');
      setTurnSignal(wantOn ? 'left' : 'off');
    });
  }

  if (rightButton) {
    rightButton.addEventListener('click', () => {
      if (isHazardOn()) {
        return; // 무시(비상등 유지)
      }

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

  // ===== 눌림 효과(pressed) =====
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

  // ===== 화살표 패드: 누르고 있는 동안만 강조 =====
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
</script>

</body>
</html>
)HTML";


String buildIndexHtml() {
    return String(FPSTR(INDEX_HTML));
}


void publishState(bool isOn) {
    const char* msg = isOn ? "ON" : "OFF";
    mqttClient.publish(TOPIC_STATE, msg, false);
}

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }
    message.trim();

    if (String(topic) == TOPIC_CMD) {
        if (message == "ON") {
            digitalWrite(LED_GPIO, HIGH);
            publishState(true);
        }
        else if (message == "OFF") {
            digitalWrite(LED_GPIO, LOW);
            publishState(false);
        }
    }
}

void connectWiFi() {
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);

    Serial.print("WiFi connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println();
    Serial.print("WiFi connected. IP: ");
    Serial.println(WiFi.localIP());
}

void connectMqtt() {
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCallback(onMqttMessage);

    while (!mqttClient.connected()) {
        String clientId = "esp32_" + String((uint32_t)ESP.getEfuseMac(), HEX);
        Serial.print("MQTT connecting... clientId=");
        Serial.println(clientId);

        if (mqttClient.connect(clientId.c_str())) {
            Serial.println("MQTT connected.");
            mqttClient.subscribe(TOPIC_CMD);
            // 초기 상태 송신
            publishState(digitalRead(LED_GPIO) == HIGH);
        }
        else {
            Serial.print("MQTT connect failed, rc=");
            Serial.println(mqttClient.state());
            delay(1000);
        }
    }
}

void setupWebServer() {
    webServer.on("/", []() {
        webServer.send(200, "text/html; charset=utf-8", buildIndexHtml());
        });

    webServer.on("/cmd", HTTP_GET, []() {
        if (!webServer.hasArg("value")) {
            webServer.send(400, "text/plain; charset=utf-8", "value 파라미터 없음");
            return;
        }

        String cmd = webServer.arg("value");
        cmd.trim();

        if (cmd == "ON") {
            digitalWrite(LED_GPIO, HIGH);
            publishState(true);
            webServer.send(200, "text/plain; charset=utf-8", "OK: ON");
            return;
        }

        if (cmd == "OFF") {
            digitalWrite(LED_GPIO, LOW);
            publishState(false);
            webServer.send(200, "text/plain; charset=utf-8", "OK: OFF");
            return;
        }

        webServer.send(400, "text/plain; charset=utf-8", "지원하지 않는 value: " + cmd);
        });

    webServer.begin();
    Serial.println("WebServer started on port 80");
}

void setup() {
    Serial.begin(115200);

    pinMode(LED_GPIO, OUTPUT);
    digitalWrite(LED_GPIO, LOW);

    connectWiFi();
    setupWebServer();

    connectMqtt();
}

void loop() {
    webServer.handleClient();

    if (!mqttClient.connected()) {
        connectMqtt();
    }
    mqttClient.loop();
}
