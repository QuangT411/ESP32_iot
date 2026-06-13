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
#include <SD.h>
#include <SPI.h>

// ===== FIREBASE =====
#define DATABASE_URL  "https://precise-irrigation-6c076-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "GrpnZk7sHKPaDkZn1oo4EiWvdUwBsqvuSYS7Lv5b"

// ===== MQTT (EMQX CLOUD) =====
#define MQTT_HOST        "ba662f8f.ala.asia-southeast1.emqxsl.com"
#define MQTT_PORT        8883
#define MQTT_USER        "quang"
#define MQTT_PASS        "12052004"
#define MQTT_TOPIC_PUMP  "irrigation/control/pump"
#define MQTT_TOPIC_TIME  "irrigation/control/time"
#define MQTT_TOPIC_STATUS "irrigation/status/pump"

// ===== RELAY =====
#define RELAY_PIN      25
#define RELAY_ON_LEVEL  HIGH
#define RELAY_OFF_LEVEL LOW

// ===== SD CARD =====  [THÊM MỚI]
#define SD_CS_PIN 5   // Chip Select pin cho SD Card (SPI)
                      // Kết nối: MOSI=23, MISO=19, SCK=18, CS=5

// ===== TRẠNG THÁI HỆ THỐNG =====  [THÊM MỚI]
bool offlineMode  = false;  // true khi không có WiFi
bool sdAvailable  = false;  // true khi SD Card sẵn sàng

// ===== FIREBASE =====
FirebaseData  fbdo;
FirebaseAuth  auth;
FirebaseConfig config;

// ===== MQTT =====
WiFiClientSecure mqttWifiClient;
PubSubClient     mqttClient(mqttWifiClient);

bool          relayOn         = false;
unsigned long relayDurationMs = 0;
unsigned long relayOffAtMs    = 0;
unsigned long lastMqttAttemptMs = 0;
const unsigned long MQTT_RECONNECT_MS = 5000;

unsigned long lastWiFiAttemptMs = 0;
const unsigned long WIFI_RECONNECT_MS = 5000;

unsigned long lastSend = 0;

// ===== NTP =====
const char* ntpServer        = "pool.ntp.org";
const long  gmtOffset_sec    = 7 * 3600;
const int   daylightOffset_sec = 0;

// ===== SENSOR =====
Adafruit_BME280 bme;
BH1750          lightMeter;

// ===== SOIL =====
const int AirValue   = 3000;
const int WaterValue = 1750;
const int SensorPin  = 34;

// ===== SOIL SMA FILTER =====
const int           SMA_WINDOW       = 10;
const unsigned long SOIL_SAMPLE_MS   = 30000UL;  // 30 giây / mẫu
float               soilSmaBuffer[SMA_WINDOW];    // circular buffer
int                 soilSmaIndex     = 0;          // vị trí ghi tiếp theo
int                 soilSmaCount     = 0;          // số mẫu hợp lệ (0..10)
unsigned long       lastSoilSampleMs = 0;

// ===== SOIL LPF (First-Order Low Pass Filter) =====
// y[n] = α·x[n] + (1-α)·y[n-1]
const float LPF_ALPHA  = 0.2f;   // α=0.2 → tin 20% SMA mới, 80% lịch sử cũ (rất mượt)
float       lpfSoilPct = -1.0f;  // -1 = chưa khởi tạo

bool servicesInited = false;

// ===== FLOW =====
const int   FLOW_PIN           = 33;
const float CALIBRATION_FACTOR = 98.0;

volatile unsigned long pulseCount   = 0;
unsigned long          lastMeasureMs = 0;

float flowRateLMin = 0.0;
float totalVolumeL = 0.0;

void IRAM_ATTR flowISR() { pulseCount++; }

// ===== WIFI PORTAL =====
WebServer server(80);
char ssid[32];
char pass[32];

// ===================== SOIL SMA(10×30s) + LPF + 5m SEND =====================
const unsigned long SEND_WINDOW_MS   = 300000;

void saveCredentials(const char* newSSID, const char* newPass)
{
  for (int i = 0; i < 32; i++) EEPROM.write(0   + i, newSSID[i]);
  for (int i = 0; i < 32; i++) EEPROM.write(100 + i, newPass[i]);
  EEPROM.commit();
}

bool readCredentials()
{
  for (int i = 0; i < 32; i++) ssid[i] = EEPROM.read(0   + i);
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
        <button type='submit'>Lưu &amp; Kết nối</button>
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

  server.on("/",     HTTP_GET,  handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();

  Serial.println("AP Portal đang chạy tại 192.168.4.1");
  while (true) {
    server.handleClient();
    delay(2);
    yield();
  }
}

bool connectWiFi()
{
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  int timeout = 20;
  while (WiFi.status() != WL_CONNECTED && timeout--) {
    delay(500);
    yield();
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void initWiFi()
{
  if (!readCredentials()) {
    // Chưa có thông tin WiFi → mở portal để người dùng nhập
    Serial.println("⚠️  Chưa có thông tin WiFi → Mở AP Portal...");
    Serial.println("    Kết nối WiFi: ESP32_IRRIGATION");
    Serial.println("    Truy cập:     http://192.168.4.1");
    startPortal(); // block tại đây, ESP32 restart sau khi lưu
  }

  // Có thông tin → thử kết nối
  Serial.print("📡 Đang kết nối WiFi: ");
  Serial.println(ssid);
  if (connectWiFi()) {
    Serial.println("✅ WiFi OK: " + WiFi.localIP().toString());
    offlineMode = false;
  } else {
    Serial.println("⚠️  WiFi FAIL → chế độ OFFLINE");
    Serial.println("    Dữ liệu sẽ lưu vào SD Card.");
    Serial.println("    Reset EEPROM để cấu hình WiFi mới nếu cần.");
    offlineMode = true;
  }
  yield();
}

//  SD CARD 
void initSD()
{
  if (SD.begin(SD_CS_PIN)) {
    sdAvailable = true;
    Serial.println("✅ SD Card OK");
  } else {
    sdAvailable = false;
    Serial.println("⚠️  SD Card FAIL e");
  }
}

// Lưu 1 bản ghi vào file CSV theo ngày (ví dụ: /DATA_20260524.csv)
void saveToSD(float t, float h, float p, float lux, float soil,
              float flow, float vol, const char* datetime)
{
  if (!sdAvailable) {
    Serial.println("SD không khả dụng, bỏ qua lưu offline.");
    return;
  }

  char filename[22];
  // datetime format: "2026-05-24 21:00:00"
  // Lấy 8 ký tự ngày (YYYY-MM-DD → YYYYMMDD) để đặt tên file
  char datepart[9] = {0};
  // datetime[0..3]=year, [5..6]=month, [8..9]=day
  snprintf(datepart, sizeof(datepart), "%c%c%c%c%c%c%c%c",
           datetime[0], datetime[1], datetime[2], datetime[3],
           datetime[5], datetime[6], datetime[8], datetime[9]);
  snprintf(filename, sizeof(filename), "/DATA_%s.csv", datepart);

  // Ghi header nếu file chưa tồn tại
  if (!SD.exists(filename)) {
    File fh = SD.open(filename, FILE_WRITE);
    if (fh) {
      fh.println("datetime,temperature,humidity,pressure_hpa,"
                 "light_lux,soil_percent,flow_L_min,total_volume_L");
      fh.close();
    }
  }

  File f = SD.open(filename, FILE_APPEND);
  if (!f) {
    Serial.println("❌ SD: mở file thất bại");
    return;
  }

  f.printf("%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n",
           datetime, t, h, p, lux, soil, flow, vol);
  f.close();
  yield();

  Serial.print("💾 Lưu offline → ");
  Serial.println(filename);
}


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
    if (msg == "ON" ||
        msg.indexOf("\"status\":\"ON\"")  >= 0 ||
        msg.indexOf("\"status\": \"ON\"") >= 0) {
      setRelayState(true);
    } else if (msg == "OFF" ||
               msg.indexOf("\"status\":\"OFF\"")  >= 0 ||
               msg.indexOf("\"status\": \"OFF\"") >= 0) {
      setRelayState(false);
    }
  } else if (strcmp(topic, MQTT_TOPIC_TIME) == 0) {
    long seconds = msg.toInt();
    if (seconds > 0) {
      relayDurationMs = (unsigned long)seconds * 1000UL;
      if (relayOn) relayOffAtMs = millis() + relayDurationMs;
      Serial.print("Relay duration (s): ");
      Serial.println(seconds);
    } else {
      relayDurationMs = 0;
      relayOffAtMs    = 0;
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
  yield();
}
void initI2C()    { Wire.begin(21, 22); }
void initLight()  { lightMeter.begin(); }
float readLight() { return lightMeter.readLightLevel(); }
void initBME280() { bme.begin(0x76); }
int readSoilRaw()
{
  return analogRead(SensorPin);
}

// Thêm 1 mẫu vào SMA buffer (gọi mỗi 30 giây)
void soilSmaPush(float pct)
{
  soilSmaBuffer[soilSmaIndex] = pct;
  soilSmaIndex = (soilSmaIndex + 1) % SMA_WINDOW;
  if (soilSmaCount < SMA_WINDOW) soilSmaCount++;
}

// Tính trung bình SMA từ các mẫu hợp lệ
float getSoilSMA()
{
  if (soilSmaCount == 0) return 0.0f;
  float sum = 0.0f;
  for (int i = 0; i < soilSmaCount; i++) sum += soilSmaBuffer[i];
  return sum / soilSmaCount;
}

float mapSoilPercentF(float soilRaw)
{
  float pct = (AirValue - soilRaw) / (float)(AirValue - WaterValue) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}

void initMqtt()
{
  mqttWifiClient.setInsecure();
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  Serial.println("✅ MQTT config");
}


void initFirebase()
{
  config.database_url                = DATABASE_URL;
  config.signer.tokens.legacy_token  = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

// Gửi trực tiếp lên Firebase (khi online)
void sendToFirebase(float t, float h, float p, float lux,
                   float soil, const char* datetime, time_t nowEpoch)
{
  // Lấy ngày từ datetime (format: "YYYY-MM-DD HH:MM:SS")
  char dateKey[11];
  strncpy(dateKey, datetime, 10);
  dateKey[10] = '\0';

  String path = "/He_thong_tuoi/sensors/data/";
  path += dateKey;
  path += "/";
  path += String(nowEpoch);

  FirebaseJson json;
  json.set("temperature",    t);
  json.set("humidity",       h);
  json.set("pressure_hpa",   p);
  json.set("soil_percent",   soil);
  json.set("light_lux",      lux);
  json.set("flow_L_min",     flowRateLMin);
  json.set("total_volume_L", totalVolumeL);
  json.set("datetime",       datetime);

  Firebase.setJSON(fbdo, path, json);
  yield();
}

void setup()
{
  Serial.begin(115200);
  EEPROM.begin(512);

  // 1. GPIO
  pinMode(RELAY_PIN, OUTPUT);
  setRelayState(false);

  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);

  // 2. Cảm biến — hoàn toàn độc lập, không cần WiFi
  initI2C();
  initLight();
  initBME280();
  Serial.println(" Cảm biến ");

  // 3. SD Card — fallback storage khi mất WiFi
  initSD();

  // 4. WiFi — có thể fail, không block nữa
  initWiFi();
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // 5. MQTT config không cần WiFi — luôn khởi tạo
  initMqtt();

  // 6. Online services — chỉ khởi tạo khi có WiFi
  if (!offlineMode) {
    initFirebase();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    servicesInited = true;
    Serial.println("✅ Firebase + NTP OK");
  } else {
    Serial.println("⚠️ Chạy offline — bỏ qua Firebase/NTP");
  }
}

void loop()
{
  // --- WiFi: tự kết nối lại khi mất ---
  if (WiFi.status() != WL_CONNECTED &&
      millis() - lastWiFiAttemptMs >= WIFI_RECONNECT_MS) {
    lastWiFiAttemptMs = millis();
    WiFi.reconnect();
  }

  // --- Cập nhật trạng thái offlineMode ---
  offlineMode = (WiFi.status() != WL_CONNECTED);

  // --- Khởi tạo Firebase + NTP lần đầu khi WiFi vừa kết nối lại ---
  if (!offlineMode && !servicesInited) {
    initFirebase();
    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    servicesInited = true;
    Serial.println("✅ WiFi khôi phục → Firebase + NTP OK");
  }

  // --- MQTT ---
  if (!offlineMode) {
    if (!mqttClient.connected() &&
        millis() - lastMqttAttemptMs >= MQTT_RECONNECT_MS) {
      lastMqttAttemptMs = millis();
      mqttConnect();
    }
    mqttClient.loop();
  }
  yield();

  // --- Relay tự tắt theo timer ---
  if (relayOn && relayOffAtMs > 0 && (long)(millis() - relayOffAtMs) >= 0) {
    setRelayState(false);
  }

  // --- FLOW ---
  if (millis() - lastMeasureMs >= 1000) {
    unsigned long now     = millis();
    unsigned long elapsed = now - lastMeasureMs;
    lastMeasureMs = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float freq    = pulses / (elapsed / 1000.0);
    // Chỉ tính flow/volume khi relay đang ON → tránh nhiễu khi chân GPIO float
    if (relayOn) {
      flowRateLMin  = freq / CALIBRATION_FACTOR;
      totalVolumeL += pulses / (CALIBRATION_FACTOR * 60.0);
    } else {
      flowRateLMin = 0.0;
    }
  }

  // --- SOIL SMA: lấy 1 mẫu mỗi 30 giây (non-blocking) ---
  if (millis() - lastSoilSampleMs >= SOIL_SAMPLE_MS) {
    lastSoilSampleMs = millis();
    float rawPct = mapSoilPercentF((float)readSoilRaw());
    soilSmaPush(rawPct);
    Serial.printf("[SMA] Mẫu %d/%d: %.2f%%\n", soilSmaCount, SMA_WINDOW, rawPct);
  }

  // --- Gửi dữ liệu mỗi 5 phút ---
  if (millis() - lastSend >= SEND_WINDOW_MS) {
    // Bước 1: SMA — trung bình 10 mẫu x 30s
    float smaSoil = getSoilSMA();

    // Bước 2: LPF — y[n] = α·x[n] + (1-α)·y[n-1]  (α=0.8)
    if (lpfSoilPct < 0.0f) lpfSoilPct = smaSoil;  // seed lần đầu
    else lpfSoilPct = LPF_ALPHA * smaSoil + (1.0f - LPF_ALPHA) * lpfSoilPct;

    float soilPercent = roundf(lpfSoilPct * 100.0f) / 100.0f;

    float t   = bme.readTemperature();
    float h   = bme.readHumidity();
    float p   = bme.readPressure() / 100.0F;
    float lux = readLight();

    time_t   nowEpoch = time(nullptr);
    struct tm ti;
    localtime_r(&nowEpoch, &ti);
    char dt[20];
    strftime(dt, sizeof(dt), "%Y-%m-%d %H:%M:%S", &ti);

    if (!offlineMode && Firebase.ready()) {
      sendToFirebase(t, h, p, lux, soilPercent, dt, nowEpoch);
      Serial.println("--- Gửi Firebase ---");
    } else {
      saveToSD(t, h, p, lux, soilPercent,
               flowRateLMin, totalVolumeL, dt);
      Serial.println("--- 💾 SD Card (offline) ---");
    }

    Serial.printf("T=%.2fC H=%.2f%% P=%.2fhPa Lux=%.2f Soil(SMA)=%.2f%% Flow=%.3f Vol=%.3f\n",
                  t, h, p, lux, soilPercent, flowRateLMin, totalVolumeL);

    lastSend = millis();
  }

  delay(1000);
}
