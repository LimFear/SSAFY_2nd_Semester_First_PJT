#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>

/* ===== Wi-Fi ===== */
const char* ssid = ""; //개인 와이파이 이름 입력
const char* password = ""; //개인 와이파이 비밀번호 입력

/* ===== MQTT Broker (EMQX Public) =====
   Host: broker.emqx.io
   TCP: 1883
   WS:  8083
   WSS: 8084  (browser uses this)  path: /mqtt
*/
const char* MQTT_HOST = "broker.emqx.io";
const uint16_t MQTT_PORT = 1883;

/* ===== Topics ===== */
const char* TOPIC_CMD   = "Lim/esp32/led27/cmd/haha5123";
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
  <meta name="viewport" content="width=device-width,initial-scale=1" />
  <title>ESP32 LED Control</title>
  <style>
    body { font-family: sans-serif; margin: 24px; }
    button { font-size: 20px; padding: 14px 18px; margin-right: 10px; }
    #status { margin-top: 14px; }
  </style>
</head>
<body>
  <h2>ESP32 LED Control</h2>

  <div>
    <button onclick="sendCmd('ON')">ON</button>
    <button onclick="sendCmd('OFF')">OFF</button>
  </div>

  <div id="status">대기 중...</div>

  <script>
    const statusEl = document.getElementById("status");

    function sendCmd(cmd) {
      statusEl.textContent = "요청 중: " + cmd;

      fetch("/cmd?value=" + encodeURIComponent(cmd))
        .then(response => response.text())
        .then(text => {
          statusEl.textContent = "응답: " + text;
        })
        .catch(error => {
          statusEl.textContent = "HTTP 에러: " + error;
        });
    }
  </script>
</body>
</html>
)HTML";


String buildIndexHtml() {
  String html = FPSTR(INDEX_HTML);
  html.replace("{{TOPIC_CMD}}", TOPIC_CMD);
  html.replace("{{TOPIC_STATE}}", TOPIC_STATE);
  return html;
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
    } else if (message == "OFF") {
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
    } else {
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
