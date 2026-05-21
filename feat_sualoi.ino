#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <time.h>
#include <PubSubClient.h>

// ===== FIREBASE =====
#define DATABASE_URL "https://precise-irrigation-6c076-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "GrpnZk7sHKPaDkZn1oo4EiWvdUwBsqvuSYS7Lv5b"

// ===== MQTT (EMQX CLOUD) =====
#define MQTT_HOST "ba662f8f.ala.asia-southeast1.emqxsl.com"
#define MQTT_PORT 8883
#define MQTT_USER "quang"
#define MQTT_PASS "12052004"
#define MQTT_TOPIC_PUMP "irrigation/control/pump"
#define MQTT_TOPIC_TIME "irrigation/control/time"
#define MQTT_TOPIC_STATUS "irrigation/status/pump"

// ===== RELAY =====
#define RELAY_PIN 33
#define RELAY_ON_LEVEL HIGH
#define RELAY_OFF_LEVEL LOW

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

WiFiClientSecure mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

bool relayOn = false;
unsigned long relayDurationMs = 0;
unsigned long relayOffAtMs = 0;
unsigned long lastMqttAttemptMs = 0;
const unsigned long MQTT_RECONNECT_MS = 5000;

unsigned long lastSend = 0;

// ===== NTP =====
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

// ===== SENSOR =====
Adafruit_BME280 bme;
BH1750 lightMeter;

// ===== SOIL =====
const int AirValue   = 3220;
const int WaterValue = 2100;
const int SensorPin  = 34;

int soilMoistureValue  = 0;
int soilmoisturepercent = 0;

// ===== FLOW =====
const int FLOW_PIN = 27;
const float CALIBRATION_FACTOR = 98.0;

volatile unsigned long pulseCount = 0;
unsigned long lastMeasureMs = 0;

float flowRateLMin  = 0.0;
float totalVolumeL  = 0.0;

void IRAM_ATTR flowISR() { pulseCount++; }

// ===== WIFI PORTAL =====
WebServer server(80);
char ssid[32];
char pass[32];

// ===================== 15s SAMPLE + 5m AVERAGE =====================
unsigned long lastSampleMs = 0;
const unsigned long SENSOR_SAMPLE_MS = 15000;
const unsigned long SEND_WINDOW_MS = 300000;

float sumT = 0, sumH = 0, sumP = 0, sumLux = 0, sumSoil = 0;
int sampleCount = 0;

float avgT = 0, avgH = 0, avgP = 0, avgLux = 0, avgSoil = 0;

// ===== WIFI (GIỮ NGUYÊN) =====
void saveCredentials(const char* newSSID, const char* newPass)
{
  for (int i = 0; i < 32; i++) EEPROM.write(0 + i, newSSID[i]);
  for (int i = 0; i < 32; i++) EEPROM.write(100 + i, newPass[i]);
  EEPROM.commit();
}

bool readCredentials()
{
  for (int i = 0; i < 32; i++) ssid[i] = EEPROM.read(0 + i);
  for (int i = 0; i < 32; i++) pass[i] = EEPROM.read(100 + i);
  ssid[31] = '\0';
  pass[31] = '\0';
  return (ssid[0] != '\0' && (uint8_t)ssid[0] != 0xFF);
}

void handleRoot()
{
  String html = R"(
    <!DOCTYPE html><html><head>
    <meta charset='UTF-8'>
    <meta name='viewport' content='width=device-width,initial-scale=1'>
    <title>WiFi Setup</title>
    <style>
      body{font-family:Arial;display:flex;justify-content:center;
           align-items:center;height:100vh;background:#f0f2f5;margin:0}
      .box{background:#fff;padding:32px;border-radius:12px;
           box-shadow:0 2px 16px rgba(0,0,0,.1);width:320px}
      h2{text-align:center;margin-bottom:20px;color:#333}
      input{width:100%;padding:10px;margin:8px 0 16px;
            border:1px solid #ddd;border-radius:8px;box-sizing:border-box;font-size:15px}
      button{width:100%;padding:12px;background:#4a90e2;color:#fff;
             border:none;border-radius:8px;font-size:16px;cursor:pointer}
    </style></head><body>
    <div class='box'>
      <h2>⚙️ Cài đặt WiFi</h2>
      <form method='POST' action='/save'>
        <label>Tên WiFi (SSID)</label>
        <input type='text' name='ssid' required>
        <label>Mật khẩu</label>
        <input type='password' name='pass'>
        <button type='submit'>Lưu & Kết nối</button>
      </form>
    </div></body></html>
  )";
  server.send(200, "text/html", html);
}

void handleSave()
{
  String newSSID = server.arg("ssid");
  String newPass = server.arg("pass");

  saveCredentials(newSSID.c_str(), newPass.c_str());

  server.send(200, "text/html",
    "<h2 style='font-family:Arial;text-align:center;margin-top:40px'>"
    "✅ Đã lưu! ESP32 đang restart...</h2>");
  delay(2000);
  ESP.restart();
}

void startPortal()
{
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32_IRRIGATION");

  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();

  while (true) {
    server.handleClient();
    delay(2);
  }
}

bool connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  int timeout = 20;
  while (WiFi.status() != WL_CONNECTED && timeout--) {
    delay(500);
  }
  return WiFi.status() == WL_CONNECTED;
}

void initWiFi()
{
  if (readCredentials() && connectWiFi()) {
    Serial.println("WiFi đã kết nối");
  } else startPortal();
}

// ===== MQTT =====
void setRelayState(bool on)
{
  relayOn = on;
  digitalWrite(RELAY_PIN, on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
  Serial.print("Relay: ");
  Serial.println(on ? "ON" : "OFF");

  if (on && relayDurationMs > 0) {
    relayOffAtMs = millis() + relayDurationMs;
  } else if (!on) {
    relayOffAtMs = 0;
  }

  if (mqttClient.connected()) {
    mqttClient.publish(MQTT_TOPIC_STATUS, on ? "ON" : "OFF", true);
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length)
{
  Serial.print("=> MQTT topic [");
  Serial.print(topic);
  Serial.print("]: ");

  String msg;
  msg.reserve(length);
  for (unsigned int i = 0; i < length; i++) {
    char c = (char)payload[i];
    Serial.print(c);
    msg += c;
  }
  Serial.println();
  msg.trim();

  if (strcmp(topic, MQTT_TOPIC_PUMP) == 0) {
    if (msg == "ON" || msg.indexOf("\"status\":\"ON\"") >= 0 || msg.indexOf("\"status\": \"ON\"") >= 0) {
      setRelayState(true);
    } else if (msg == "OFF" || msg.indexOf("\"status\":\"OFF\"") >= 0 || msg.indexOf("\"status\": \"OFF\"") >= 0) {
      setRelayState(false);
    }
  } else if (strcmp(topic, MQTT_TOPIC_TIME) == 0) {
    long seconds = msg.toInt();
    if (seconds > 0) {
      relayDurationMs = (unsigned long)seconds * 1000UL;
      if (relayOn) {
        relayOffAtMs = millis() + relayDurationMs;
      }
      Serial.print("Relay duration (s): ");
      Serial.println(seconds);
    } else {
      relayDurationMs = 0;
      relayOffAtMs = 0;
      Serial.println("Relay duration cleared");
    }
  }
}

void mqttConnect()
{
  if (mqttClient.connected()) return;

  String clientId = "ESP32_Receiver_";
  clientId += String(random(0xffff), HEX);

  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    mqttClient.subscribe(MQTT_TOPIC_PUMP);
    mqttClient.subscribe(MQTT_TOPIC_TIME);
    mqttClient.publish(MQTT_TOPIC_STATUS, relayOn ? "ON" : "OFF", true);
    Serial.println("MQTT connected");
  } else {
    Serial.print("MQTT connect failed, rc=");
    Serial.println(mqttClient.state());
  }
}

// ===== SENSOR =====
void initI2C() { Wire.begin(21, 22); }

void initLight() { lightMeter.begin(); }

float readLight() { return lightMeter.readLightLevel(); }

void initBME280() { bme.begin(0x76); }

// ===== FIREBASE =====
void initFirebase()
{
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

// ===== SETUP =====
void setup()
{
  Serial.begin(115200);
  EEPROM.begin(512);

  pinMode(RELAY_PIN, OUTPUT);
  setRelayState(false);

  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);

  initI2C();
  initLight();
  initBME280();
  initWiFi();
  initFirebase();

  mqttWifiClient.setInsecure();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
}

// ===== LOOP =====
void loop()
{
  if (!mqttClient.connected() && millis() - lastMqttAttemptMs >= MQTT_RECONNECT_MS) {
    lastMqttAttemptMs = millis();
    mqttConnect();
  }
  mqttClient.loop();
  yield();

  if (relayOn && relayOffAtMs > 0 && (long)(millis() - relayOffAtMs) >= 0) {
    setRelayState(false);
  }

  // FLOW (GIỮ NGUYÊN)
  if (millis() - lastMeasureMs >= 1000) {
    unsigned long now = millis();
    unsigned long elapsed = now - lastMeasureMs;
    lastMeasureMs = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float freq = pulses / (elapsed / 1000.0);
    flowRateLMin = freq / CALIBRATION_FACTOR;

    totalVolumeL += pulses / (CALIBRATION_FACTOR * 60.0);
  }

  // ===== SENSOR (15s/sample) =====
  if (millis() - lastSampleMs >= SENSOR_SAMPLE_MS) {
    lastSampleMs = millis();

    int soilRaw = analogRead(SensorPin);
    int soilPercent = map(soilRaw, AirValue, WaterValue, 0, 100);
    soilPercent = constrain(soilPercent, 0, 100);

    float t = bme.readTemperature();
    float h = bme.readHumidity();
    float p = bme.readPressure() / 100.0F; // hPa
    float lux = readLight();

    sumSoil += soilPercent;
    sumT += t;
    sumH += h;
    sumP += p;
    sumLux += lux;
    sampleCount++;

    Serial.print("Sample #");
    Serial.print(sampleCount);
    Serial.print(" | T=");
    Serial.print(t, 2);
    Serial.print("C H=");
    Serial.print(h, 2);
    Serial.print("% P=");
    Serial.print(p, 2);
    Serial.print("hPa Lux=");
    Serial.print(lux, 2);
    Serial.print(" Soil=");
    Serial.print(soilPercent);
    Serial.println("%");
  }

  // ===== FIREBASE (5 phút, trung bình) =====
  if (millis() - lastSend >= SEND_WINDOW_MS) {
    if (sampleCount > 0) {
      avgSoil = sumSoil / sampleCount;
      avgT    = sumT / sampleCount;
      avgH    = sumH / sampleCount;
      avgP    = sumP / sampleCount;
      avgLux  = sumLux / sampleCount;

      time_t nowEpoch = time(nullptr);
      String path = "/He_thong_tuoi/sensors/data/" + String(nowEpoch);

      struct tm timeinfo;
      localtime_r(&nowEpoch, &timeinfo);
      char datetimeStr[20];
      strftime(datetimeStr, sizeof(datetimeStr), "%Y-%m-%d %H:%M:%S", &timeinfo);

      FirebaseJson json;
      json.set("temperature", avgT);
      json.set("humidity", avgH);
      json.set("pressure_hpa", avgP);
      json.set("soil_percent", avgSoil);
      json.set("light_lux", avgLux);
      json.set("flow_L_min", flowRateLMin);
      json.set("total_volume_L", totalVolumeL);
      json.set("datetime", datetimeStr);
      Firebase.setJSON(fbdo, path, json);
      yield();

      Serial.println("--- 5m AVG SENT ---");
      Serial.print("Avg T=");
      Serial.print(avgT, 2);
      Serial.print("C Avg H=");
      Serial.print(avgH, 2);
      Serial.print("% Avg P=");
      Serial.print(avgP, 2);
      Serial.print("hPa Avg Lux=");
      Serial.print(avgLux, 2);
      Serial.print(" Avg Soil=");
      Serial.print(avgSoil, 2);
      Serial.println("%");
      Serial.print("Flow L/min=");
      Serial.print(flowRateLMin, 3);
      Serial.print(" Total L=");
      Serial.println(totalVolumeL, 3);

      sumSoil = sumT = sumH = sumP = sumLux = 0;
      sampleCount = 0;
      lastSend = millis();
    }
  }

  delay(1000);
}
