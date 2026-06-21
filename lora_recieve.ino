
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <WebServer.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <FirebaseESP32.h>
#include <addons/TokenHelper.h>
#include <addons/RTDBHelper.h>
#include <PubSubClient.h>
#include <time.h>

// LoRa SX1278 (433MHz) 
#define LORA_NSS 5
#define LORA_RST 14
#define LORA_DIO0 2
#define LORA_FREQ 433E6

// ===== FIREBASE =====
#define DATABASE_URL "https://precise-irrigation-6c076-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "GrpnZk7sHKPaDkZn1oo4EiWvdUwBsqvuSYS7Lv5b"

//MQTT (EMQX CLOUD) 
#define MQTT_HOST "ba662f8f.ala.asia-southeast1.emqxsl.com"
#define MQTT_PORT 8883
#define MQTT_USER "quang"
#define MQTT_PASS "12052004"
#define MQTT_TOPIC_PUMP "irrigation/device1/control/pump"
#define MQTT_TOPIC_TIME "irrigation/device1/control/time"
#define MQTT_TOPIC_STATUS "irrigation/device1/status/pump"

//  FIREBASE 
FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

//  MQTT 
WiFiClientSecure mqttWifiClient;
PubSubClient mqttClient(mqttWifiClient);

bool pumpOn = false;              
unsigned long relayDurationMs = 0; 
unsigned long lastMqttAttemptMs = 0;
const unsigned long MQTT_RECONNECT_MS = 5000;

unsigned long lastWiFiAttemptMs = 0;
const unsigned long WIFI_RECONNECT_MS = 5000;

bool offlineMode = false;

// ===== OLED CONFIG & LATEST SENSOR DATA =====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

float latestTemp = -1.0f;
float latestHum  = -1.0f;
float latestSoil = -1.0f;
float latestLux  = -1.0f;

unsigned long lastOledUpdateMs = 0;
const unsigned long OLED_UPDATE_INTERVAL_MS = 1000;

void initOLED() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("❌ OLED SSD1306 khởi tạo thất bại!"));
  } else {
    Serial.println(F("✅ OLED SSD1306 OK"));
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
    display.setCursor(6, 18);
    display.println("GATEWAY STARTING...");
    display.display();
  }
}

String getDisplayTimeString() {
  struct tm timeinfo;
  if (getLocalTime(&timeinfo) && timeinfo.tm_year > 70) {
    char buffer[22];
    snprintf(buffer, sizeof(buffer), "%02d/%02d/%04d %02d:%02d:%02d",
             timeinfo.tm_mday, timeinfo.tm_mon + 1, timeinfo.tm_year + 1900,
             timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return String(buffer);
  }
  return "--/--/---- --:--:--";
}

void updateOLEDDisplay() {
  display.clearDisplay();
  
  // Khung viền xung quanh màn hình
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  
  // Header: Trạng thái kết nối
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(6, 2);
  
  String wifiStat = "OFF";
  if (WiFi.status() == WL_CONNECTED) {
    wifiStat = "CON";
  }
  String mqttStat = mqttClient.connected() ? "OK" : "--";
  
  char header[22];
  snprintf(header, sizeof(header), "GW MON  [W:%s M:%s]", wifiStat.c_str(), mqttStat.c_str());
  display.print(header);
  
  // Đường kẻ ngang phân tách Header
  display.drawFastHLine(4, 11, 120, SSD1306_WHITE);
  
  // Hiển thị tất cả thông số trên một màn hình duy nhất
  // Sử dụng 2 cột để chứa đủ các thông số
  
  // 1. Cột 1: Nhiệt độ & Độ ẩm
  display.setCursor(6, 15);
  display.print("T: ");
  if (latestTemp < 0.0f) {
    display.print("--.-");
  } else {
    display.print(latestTemp, 1);
  }
  display.write(247); // Ký tự độ C
  display.print("C");

  display.setCursor(68, 15);
  display.print("H: ");
  if (latestHum < 0.0f) {
    display.print("--.-");
  } else {
    display.print(latestHum, 1);
  }
  display.print("%");

  // 2. Cột 2: Độ ẩm đất & Ánh sáng (Lux)
  display.setCursor(6, 27);
  display.print("S: ");
  if (latestSoil < 0.0f) {
    display.print("--.-");
  } else {
    display.print(latestSoil, 1);
  }
  display.print("%");

  display.setCursor(68, 27);
  display.print("L: ");
  if (latestLux < 0.0f) {
    display.print("---");
  } else {
    display.print((int)latestLux);
  }
  display.print(" lx");

  // 3. Trạng thái bơm (được đưa ra giữa dòng)
  char pumpStr[25];
  if (pumpOn) {
    if (relayDurationMs > 0) {
      snprintf(pumpStr, sizeof(pumpStr), "Pump: ON (T)");
    } else {
      snprintf(pumpStr, sizeof(pumpStr), "Pump: ON");
    }
  } else {
    snprintf(pumpStr, sizeof(pumpStr), "Pump: OFF");
  }
  int pumpX = (128 - (strlen(pumpStr) * 6)) / 2;
  display.setCursor(pumpX, 39);
  display.print(pumpStr);
  
  // Đường kẻ ngang phân tách Footer
  display.drawFastHLine(4, 50, 120, SSD1306_WHITE);
  
  // Footer: Hiển thị thời gian đồng bộ từ NTP Server (được căn giữa)
  String timeStr = getDisplayTimeString();
  int timeX = (128 - (timeStr.length() * 6)) / 2;
  display.setCursor(timeX, 53);
  display.print(timeStr);
  
  display.display();
}

// ===== NTP =====
const char* ntpServer = "pool.ntp.org";
const long gmtOffset_sec = 7 * 3600;
const int daylightOffset_sec = 0;

// ===== WIFI PORTAL (EEPROM) =====
WebServer server(80);
char ssid[32];
char pass[32];

//  EEPROM — lưu/đọc WiFi credentials
void saveCredentials(const char* newSSID, const char* newPass) {
  for (int i = 0; i < 32; i++) EEPROM.write(0 + i, newSSID[i]);
  for (int i = 0; i < 32; i++) EEPROM.write(100 + i, newPass[i]);
  EEPROM.commit();
}

bool readCredentials() {
  for (int i = 0; i < 32; i++) ssid[i] = EEPROM.read(0 + i);
  for (int i = 0; i < 32; i++) pass[i] = EEPROM.read(100 + i);
  ssid[31] = '\0';
  pass[31] = '\0';
  return (ssid[0] != '\0' && (uint8_t)ssid[0] != 0xFF);
}

//  WIFI CAPTIVE PORTAL

void handleRoot() {
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
      <h2>⚙️ Cài đặt WiFi (Gateway)</h2>
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

void handleSave() {
  String newSSID = server.arg("ssid");
  String newPass = server.arg("pass");
  saveCredentials(newSSID.c_str(), newPass.c_str());
  server.send(200, "text/html",
              "<h2 style='font-family:Arial;text-align:center;margin-top:40px'>"
              "✅ Đã lưu! ESP32 đang restart...</h2>");
  delay(2000);
  ESP.restart();
}

void startPortal() {
  WiFi.mode(WIFI_AP);
  WiFi.softAP("ESP32_GATEWAY");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/save", HTTP_POST, handleSave);
  server.begin();
  Serial.println("📶 AP Portal chạy tại 192.168.4.1 (SSID: ESP32_GATEWAY)");
  
  // Hiển thị thông báo Captive Portal lên màn hình OLED
  display.clearDisplay();
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(6, 4);
  display.print("   WIFI PORTAL");
  display.drawFastHLine(4, 14, 120, SSD1306_WHITE);
  
  display.setCursor(6, 18);
  display.print("SSID: ESP32_GATEWAY");
  display.setCursor(6, 29);
  display.print("IP  : 192.168.4.1");
  display.setCursor(6, 40);
  display.print("Configure WiFi...");
  
  display.drawFastHLine(4, 51, 120, SSD1306_WHITE);
  display.setCursor(6, 54);
  display.print("AP Mode Active");
  display.display();

  while (true) {
    server.handleClient();
    delay(2);
  }
}
bool connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  int timeout = 20;
  while (WiFi.status() != WL_CONNECTED && timeout--) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  return WiFi.status() == WL_CONNECTED;
}

void initWiFi() {
  if (readCredentials() && connectWiFi()) {
    Serial.println("✅ WiFi OK: " + WiFi.localIP().toString());
    offlineMode = false;
  } else {
    Serial.println("⚠️  Không kết nối được WiFi → mở Portal để cài đặt...");
    startPortal(); 
  }
}

//  LORA INIT
void initLoRa() {
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("❌ LoRa khởi tạo thất bại! Kiểm tra kết nối dây.");
    while (true) { delay(1000); }
  }
  LoRa.setSpreadingFactor(12);                     // SF12: tầm xa tối đa
  LoRa.setSignalBandwidth(125E3);                   // BW125kHz
  LoRa.setCodingRate4(8);                           // CR4/8: chống nhiễu tốt nhất
  LoRa.setTxPower(18, PA_OUTPUT_PA_BOOST_PIN);      // 18dBm — max theo spec module phần cứng
  LoRa.setPreambleLength(12);                       // Preamble dài hơn để bắt sóng dễ hơn
  LoRa.enableCrc();
  Serial.println("✅ LoRa OK (433MHz, SF12, BW125, CR4/8, 18dBm)");
}

//  FIREBASE
void initFirebase() {
  config.database_url = DATABASE_URL;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;
  Firebase.begin(&config, &auth);
  Firebase.reconnectNetwork(true);
}

void sendToFirebase(float t, float h, float p, float lux,
                    float soil, float flow, float vol,
                    const char* datetime, time_t nowEpoch) {
  const char* nameDevice = "device1";
  String path = "He_thong_tuoi/sensor/" + String(nameDevice) + "/data/" + String(nowEpoch);

  FirebaseJson json;
  json.set("temperature", t);
  json.set("humidity", h);
  json.set("pressure_hpa", p);
  json.set("soil_percent", soil);
  json.set("light_lux", lux);
  json.set("flow_L_min", flow);
  json.set("total_volume_L", vol);
  json.set("datetime", datetime);

  if (Firebase.setJSON(fbdo, path, json)) {
    Serial.println("✅ Firebase OK → " + String(datetime));
  } else {
    Serial.println("❌ Firebase error: " + fbdo.errorReason());
  }
  yield();
}

//  GỬI LỆNH BƠM QUA LORA → LORA_SEND
void sendPumpCommand(const char* state, long durSeconds) {
  StaticJsonDocument<128> doc;
  doc["cmd"]   = "PUMP";
  doc["state"] = state;
  doc["dur"]   = durSeconds;

  String payload;
  serializeJson(doc, payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket(true);  // async=true — không block

  Serial.print("📡 LoRa → lệnh bơm: ");
  Serial.println(payload);

  // Không publish MQTT status ở đây.
  // Status sẽ được publish sau khi nhận ACK từ lora_send.
  pumpOn = (strcmp(state, "ON") == 0);  // cập nhật củc bộ để tham khảo
}

//  MQTT CALLBACK — nhận lệnh từ app, forward qua LoRa
void mqttCallback(char* topic, byte* payload, unsigned int length) {
  Serial.print("=> MQTT [");
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
    bool isOn = (msg == "ON" || msg.indexOf("\"status\":\"ON\"") >= 0 || msg.indexOf("\"status\": \"ON\"") >= 0);
    bool isOff = (msg == "OFF" || msg.indexOf("\"status\":\"OFF\"") >= 0 || msg.indexOf("\"status\": \"OFF\"") >= 0);

    if (isOn) {
      sendPumpCommand("ON", relayDurationMs / 1000);
    } else if (isOff) {
      sendPumpCommand("OFF", 0);
    }

  } else if (strcmp(topic, MQTT_TOPIC_TIME) == 0) {
    long seconds = msg.toInt();
    if (seconds > 0) {
      relayDurationMs = (unsigned long)seconds * 1000UL;
      Serial.print("⏱️  Thời gian bơm (s): ");
      Serial.println(seconds);
    } else {
      relayDurationMs = 0;
      Serial.println("⏱️  Đã xóa thời gian bơm");
    }
  }
}

void mqttConnect() {
  if (mqttClient.connected()) return;
  String clientId = "ESP32_Gateway_" + String(random(0xffff), HEX);
  if (mqttClient.connect(clientId.c_str(), MQTT_USER, MQTT_PASS)) {
    mqttClient.subscribe(MQTT_TOPIC_PUMP);
    mqttClient.subscribe(MQTT_TOPIC_TIME);
    mqttClient.publish(MQTT_TOPIC_STATUS, pumpOn ? "ON" : "OFF", true);
    Serial.println("✅ MQTT connected");
  } else {
    Serial.print("❌ MQTT failed, rc=");
    Serial.println(mqttClient.state());
  }
}
//  NHẬN DATA CẢM BIẾN HOẶC ACK TỪ LORA_SEND
void checkLoRaData() {
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  String received = "";
  while (LoRa.available()) {
    received += (char)LoRa.read();
  }

  int rssi = LoRa.packetRssi();
  Serial.print("📥 LoRa ← (RSSI=");
  Serial.print(rssi);
  Serial.print("dBm): ");
  Serial.println(received);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, received);
  if (err) {
    Serial.print("⚠️  JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  // --- Xử LÝ ACK trạng thái bơm từ lora_send ---
  if (doc.containsKey("ack") && strcmp(doc["ack"] | "", "PUMP") == 0) {
    const char* st = doc["state"] | "OFF";
    pumpOn = (strcmp(st, "ON") == 0);
    Serial.print("✅ ACK bơm nhận được: ");
    Serial.println(st);
    if (mqttClient.connected()) {
      mqttClient.publish(MQTT_TOPIC_STATUS, st, true);
      Serial.print("📤 MQTT publish status: ");
      Serial.println(st);
    }
    return;  // không xử lý tiếp
  }

  // --- Xử LÝ DATA cảm biến từ lora_send ---
  float t    = doc["t"]    | -1.0f;
  float h    = doc["h"]    | -1.0f;
  float p    = doc["p"]    | -1.0f;
  float lux  = doc["lux"]  | -1.0f;
  float soil = doc["soil"] | -1.0f;
  float flow = doc["flow"] | 0.0f;
  float vol  = doc["vol"]  | 0.0f;

  latestTemp = t;
  latestHum  = h;
  latestSoil = soil;
  latestLux  = lux;

  Serial.printf("   T=%.2fC H=%.2f%% P=%.2fhPa Lux=%.2f Soil=%.2f%% Flow=%.3fL/m Vol=%.3fL\n",
                t, h, p, lux, soil, flow, vol);

  if (!offlineMode && Firebase.ready()) {
    time_t nowEpoch = time(nullptr);
    struct tm ti;
    localtime_r(&nowEpoch, &ti);
    char dt[20];
    strftime(dt, sizeof(dt), "%Y-%m-%d %H:%M:%S", &ti);
    sendToFirebase(t, h, p, lux, soil, flow, vol, dt, nowEpoch);
  } else {
    Serial.println("⚠️  Offline / Firebase chưa sẵn sàng — bỏ qua lần này");
  }
}


void setup() {
  Serial.begin(115200);
  EEPROM.begin(512);

  // Khởi tạo I2C và màn hình OLED
  Wire.begin(21, 22);
  initOLED();

  // WiFi
  initWiFi();
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);

  // LoRa — luôn khởi tạo, kể cả offline
  initLoRa();

  // Online services
  if (!offlineMode) {
    initFirebase();

    mqttWifiClient.setInsecure();
    mqttClient.setServer(MQTT_HOST, MQTT_PORT);
    mqttClient.setCallback(mqttCallback);

    configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
    Serial.println("✅ Firebase + MQTT + NTP OK");
  } else {
    Serial.println(" Offline — bỏ qua Firebase/MQTT/NTP");
  }
}

//  LOOP
void loop() {
  // --- WiFi tự kết nối lại khi mất ---
  if (WiFi.status() != WL_CONNECTED && millis() - lastWiFiAttemptMs >= WIFI_RECONNECT_MS) {
    lastWiFiAttemptMs = millis();
    WiFi.reconnect();
  }

  // --- Cập nhật offlineMode ---
  offlineMode = (WiFi.status() != WL_CONNECTED);

  // --- MQTT ---
  if (!offlineMode) {
    if (!mqttClient.connected() && millis() - lastMqttAttemptMs >= MQTT_RECONNECT_MS) {
      lastMqttAttemptMs = millis();
      mqttConnect();
    }
    mqttClient.loop();
  }
  checkLoRaData();

  // --- Cập nhật màn hình OLED mỗi giây ---
  if (millis() - lastOledUpdateMs >= OLED_UPDATE_INTERVAL_MS) {
    lastOledUpdateMs = millis();
    updateOLEDDisplay();
  }

  yield();
  delay(10);
}
