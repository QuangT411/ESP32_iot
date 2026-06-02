#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SD.h>

// ===== LoRa SX1278 (433MHz) =====
#define LORA_NSS   5
#define LORA_RST   14
#define LORA_DIO0  2
#define LORA_FREQ  433E6

// ===== SD CARD =====
// Dùng CS riêng (pin 4) để tránh xung đột với LoRa NSS (pin 5)
// Kết nối: MOSI=23, MISO=19, SCK=18, CS=4
#define SD_CS_PIN  4
bool sdAvailable = false;

// ===== RELAY / BƠM =====
#define RELAY_PIN       25
#define RELAY_ON_LEVEL  HIGH
#define RELAY_OFF_LEVEL LOW

// ===== SOIL MOISTURE =====
const int AirValue   = 3000;
const int WaterValue = 1750;
const int SensorPin  = 34;
const float SOIL_ALPHA = 0.2f;
float soilFilteredRaw = 0.0f;
bool  soilFilteredInit = false;
int   lastSoilRaw = -1;

// ===== FLOW METER =====
const int   FLOW_PIN           = 27;
const float CALIBRATION_FACTOR = 98.0;

volatile unsigned long pulseCount    = 0;
unsigned long          lastMeasureMs = 0;

float flowRateLMin = 0.0;
float totalVolumeL = 0.0;

void IRAM_ATTR flowISR() { pulseCount++; }

// ===== RELAY STATE =====
bool          relayOn         = false;
unsigned long relayDurationMs = 0;   
unsigned long relayOffAtMs    = 0;

// ===== SENSOR =====
Adafruit_BME280 bme;
BH1750  lightMeter;

// ===== SEND =====
unsigned long lastSendMs   = 0;
const unsigned long SEND_WINDOW_MS   = 300000;  // 5 phút

// ===================================================================
//  SOIL MEDIAN(9) — raw -> median -> map
// ===================================================================
int readSoilRaw()
{
  return analogRead(SensorPin);
}

float mapSoilPercentF(float soilRaw)
{
  float pct = (AirValue - soilRaw) / (float)(AirValue - WaterValue) * 100.0f;
  return constrain(pct, 0.0f, 100.0f);
}

int readSoilMedianRaw9()
{
  int values[9];
  for (int i = 0; i < 9; i++) {
    values[i] = readSoilRaw();
    if (i < 8) delay(100);
  }

  for (int i = 1; i < 9; i++) {
    int key = values[i];
    int j = i - 1;
    while (j >= 0 && values[j] > key) {
      values[j + 1] = values[j];
      j--;
    }
    values[j + 1] = key;
  }

  return values[4];
}


// ===================================================================
//  SD CARD
// ===================================================================
void initSD()
{
  if (SD.begin(SD_CS_PIN)) {
    sdAvailable = true;
    Serial.println("✅ SD Card OK");
  } else {
    sdAvailable = false;
    Serial.println("⚠️  SD Card FAIL — sẽ không lưu offline");
  }
}

void saveToSD(float t, float h, float p, float lux, float soil,
              float flow, float vol)
{
  if (!sdAvailable) return;

  const char* filename = "/FIELD_DATA.csv";

  // Ghi header nếu file chưa tồn tại
  if (!SD.exists(filename)) {
    File fh = SD.open(filename, FILE_WRITE);
    if (fh) {
      fh.println("temperature,humidity,pressure_hpa,"
                 "light_lux,soil_percent,flow_L_min,total_volume_L");
      fh.close();
    }
  }

  File f = SD.open(filename, FILE_APPEND);
  if (!f) {
    Serial.println("❌ SD: mở file thất bại");
    return;
  }

  f.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.3f,%.3f\n",
           t, h, p, lux, soil, flow, vol);
  f.close();

  Serial.println("💾 SD lưu OK");
}

// ===================================================================
//  LoRa INIT
// ===================================================================
void initLoRa()
{
  LoRa.setPins(LORA_NSS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("❌ LoRa khởi tạo thất bại! Kiểm tra kết nối dây.");
    while (true) { delay(1000); }
  }
  LoRa.setSpreadingFactor(9);
  LoRa.setSignalBandwidth(125E3);
  LoRa.setCodingRate4(5);
  LoRa.enableCrc();
  Serial.println("✅ LoRa OK (433MHz, SF9, BW125, CR4/5)");
}

// ===================================================================
//  RELAY
// ===================================================================
void setRelayState(bool on)
{
  relayOn = on;
  digitalWrite(RELAY_PIN, on ? RELAY_ON_LEVEL : RELAY_OFF_LEVEL);
  Serial.print("💧 Relay/Bơm: ");
  Serial.println(on ? "ON" : "OFF");

  if (on && relayDurationMs > 0) {
    relayOffAtMs = millis() + relayDurationMs;
    Serial.print("   Tự tắt sau (s): ");
    Serial.println(relayDurationMs / 1000);
  } else if (!on) {
    relayOffAtMs = 0;
  }
}

// ===================================================================
//  GỬI DATA CẢM BIẾN QUA LORA
// ===================================================================
void sendSensorData(float t, float h, float p, float lux, float soil)
{
  StaticJsonDocument<256> doc;
  doc["t"]    = t;
  doc["h"]    = h;
  doc["p"]    = p;
  doc["lux"]  = lux;
  doc["soil"] = soil;
  doc["flow"] = flowRateLMin;
  doc["vol"]  = totalVolumeL;

  String payload;
  serializeJson(doc, payload);

  LoRa.beginPacket();
  LoRa.print(payload);
  LoRa.endPacket(true);

  Serial.print("📡 LoRa → gửi sensor: ");
  Serial.println(payload);
}

void checkLoRaCommand()
{
  int packetSize = LoRa.parsePacket();
  if (packetSize == 0) return;

  String received = "";
  while (LoRa.available()) {
    received += (char)LoRa.read();
  }

  int rssi = LoRa.packetRssi();
  Serial.print("📥 LoRa ← lệnh bơm (RSSI=");
  Serial.print(rssi);
  Serial.print("dBm): ");
  Serial.println(received);

  StaticJsonDocument<128> doc;
  DeserializationError err = deserializeJson(doc, received);
  if (err) {
    Serial.print("⚠️  JSON parse error: ");
    Serial.println(err.c_str());
    return;
  }

  const char* cmd = doc["cmd"] | "";
  if (strcmp(cmd, "PUMP") != 0) return;  // bỏ qua lệnh không rõ

  const char* state = doc["state"] | "OFF";
  long        dur   = doc["dur"]   | 0;

  relayDurationMs = (dur > 0) ? (unsigned long)dur * 1000UL : 0;

  if (strcmp(state, "ON") == 0) {
    setRelayState(true);
  } else {
    setRelayState(false);
  }
}

// ===================================================================
//  SETUP
// ===================================================================
void setup()
{
  Serial.begin(115200);

  // GPIO
  pinMode(RELAY_PIN, OUTPUT);
  setRelayState(false);

  pinMode(FLOW_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), flowISR, RISING);

  // I2C + Cảm biến
  Wire.begin(21, 22);
  lightMeter.begin();
  if (!bme.begin(0x76)) {
    Serial.println("⚠️  BME280 không tìm thấy (kiểm tra địa chỉ I2C)");
  }
  Serial.println("✅ Cảm biến OK (BME280 + BH1750 + Soil + Flow)");

  // SD Card
  initSD();

  // LoRa
  initLoRa();

  lastSendMs   = millis();
  lastMeasureMs = millis();
}

// ===================================================================
//  LOOP
// ===================================================================
void loop()
{
  // --- Lắng nghe lệnh bơm từ lora_recieve ---
  checkLoRaCommand();

  // --- Relay tự tắt theo timer ---
  if (relayOn && relayOffAtMs > 0 && (long)(millis() - relayOffAtMs) >= 0) {
    Serial.println("⏰ Hết giờ → tắt bơm tự động");
    setRelayState(false);
  }

  // --- Đo flow meter mỗi giây ---
  if (millis() - lastMeasureMs >= 1000) {
    unsigned long now     = millis();
    unsigned long elapsed = now - lastMeasureMs;
    lastMeasureMs = now;

    noInterrupts();
    unsigned long pulses = pulseCount;
    pulseCount = 0;
    interrupts();

    float freq    = pulses / (elapsed / 1000.0f);
    flowRateLMin  = freq / CALIBRATION_FACTOR;
    totalVolumeL += pulses / (CALIBRATION_FACTOR * 60.0f);
  }

  // --- READ BME/BH1750 ONCE + SOIL MEDIAN(9) mỗi 5 phút ---
  if (millis() - lastSendMs >= SEND_WINDOW_MS) {
    int soilMedianRaw = readSoilMedianRaw9();

    float alpha = SOIL_ALPHA;
    if (lastSoilRaw >= 0) {
      int deltaAdc = lastSoilRaw - soilMedianRaw;
      if (deltaAdc >= 100) {
        alpha = 0.8f;
      }
    }

    if (!soilFilteredInit) {
      soilFilteredRaw = (float)soilMedianRaw;
      soilFilteredInit = true;
    } else {
      soilFilteredRaw = alpha * (float)soilMedianRaw + (1.0f - alpha) * soilFilteredRaw;
    }

    lastSoilRaw = soilMedianRaw;

    float soilPercent = mapSoilPercentF(soilFilteredRaw);
    soilPercent = roundf(soilPercent * 100.0f) / 100.0f;

    float t   = bme.readTemperature();
    float h   = bme.readHumidity();
    float p   = bme.readPressure() / 100.0F;
    float lux = lightMeter.readLightLevel();

    sendSensorData(t, h, p, lux, soilPercent);
    saveToSD(t, h, p, lux, soilPercent, flowRateLMin, totalVolumeL);

    Serial.printf("T=%.2fC H=%.2f%% P=%.2fhPa Lux=%.2f Soil=%.2f%% Flow=%.3fL/m Vol=%.3fL\n",
            t, h, p, lux, soilPercent, flowRateLMin, totalVolumeL);

    lastSendMs  = millis();
  }

  delay(10);
}
