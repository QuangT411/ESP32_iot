#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoJson.h>
#include <BH1750.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SD.h>
#include <esp_task_wdt.h>

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
BH1750          lightMeter;

// ===== SAMPLING / SEND =====
unsigned long lastSampleMs = 0;
unsigned long lastSendMs   = 0;
const unsigned long SENSOR_SAMPLE_MS = 15000;   // 15 giây
const unsigned long SEND_WINDOW_MS   = 300000;  // 5 phút

float sumT = 0, sumH = 0, sumP = 0, sumLux = 0;
int   soilSamples[30];
int   sampleCount = 0;

// ===================================================================
//  MEDIAN FILTER — lọc nhiễu cảm biến đất
// ===================================================================
#define MEDIAN_SAMPLES 11  // số lẻ để có giá trị giữa rõ ràng

int readSoilMedian()
{
  int buf[MEDIAN_SAMPLES];

  // Thu thập mẫu
  for (int i = 0; i < MEDIAN_SAMPLES; i++) {
    buf[i] = analogRead(SensorPin);
    delay(2);
  }

  // Sắp xếp tăng dần (insertion sort — nhỏ gọn cho mảng ngắn)
  for (int i = 1; i < MEDIAN_SAMPLES; i++) {
    int key = buf[i];
    int j   = i - 1;
    while (j >= 0 && buf[j] > key) {
      buf[j + 1] = buf[j];
      j--;
    }
    buf[j + 1] = key;
  }

  return buf[MEDIAN_SAMPLES / 2];  // giá trị giữa
}

// Tính Median cho một mảng số nguyên
float getMedian(int arr[], int n)
{
  // Sắp xếp tăng dần (insertion sort)
  for (int i = 1; i < n; i++) {
    int key = arr[i];
    int j   = i - 1;
    while (j >= 0 && arr[j] > key) {
      arr[j + 1] = arr[j];
      j--;
    }
    arr[j + 1] = key;
  }

  if (n % 2 == 1) {
    return arr[n / 2];
  } else {
    return (arr[n / 2 - 1] + arr[n / 2]) / 2.0;
  }
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

  esp_task_wdt_reset();  // tránh watchdog khi SD card chậm
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

// ===================================================================
//  NHẬN LỆNH BƠM TỪ LORA_RECIEVE
// ===================================================================
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

  // Watchdog
  esp_task_wdt_init(15, true);
  esp_task_wdt_add(NULL);
  Serial.println("✅ Watchdog OK (timeout=15s)");

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

  lastSampleMs = millis();
  lastSendMs   = millis();
  lastMeasureMs = millis();
}

// ===================================================================
//  LOOP
// ===================================================================
void loop()
{
  esp_task_wdt_reset();

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

  // --- Sample cảm biến mỗi 15 giây ---
  if (millis() - lastSampleMs >= SENSOR_SAMPLE_MS) {
    lastSampleMs = millis();

    // Đọc soil moisture: Median Filter 11 mẫu — chống spike hoàn toàn
    int soilRaw     = readSoilMedian();
    int soilPercent = map(soilRaw, AirValue, WaterValue, 0, 100);
    soilPercent     = constrain(soilPercent, 0, 100);

    float t   = bme.readTemperature();
    float h   = bme.readHumidity();
    float p   = bme.readPressure() / 100.0F;
    float lux = lightMeter.readLightLevel();

    if (sampleCount < 30) {
      soilSamples[sampleCount] = soilPercent;
    }
    sumT    += t;
    sumH    += h;
    sumP    += p;
    sumLux  += lux;
    sampleCount++;

    Serial.printf("Sample #%d | T=%.2fC H=%.2f%% P=%.2fhPa Lux=%.2f Soil=%d%% Flow=%.3fL/m\n",
                  sampleCount, t, h, p, lux, soilPercent, flowRateLMin);
  }

  // --- Gửi trung bình 5 phút qua LoRa ---
  if (millis() - lastSendMs >= SEND_WINDOW_MS && sampleCount > 0) {
    float avgT    = sumT    / sampleCount;
    float avgH    = sumH    / sampleCount;
    float avgP    = sumP    / sampleCount;
    float avgLux  = sumLux  / sampleCount;

    // Tính median cho độ ẩm đất (soil moisture) sau 5 phút
    int validSamples = (sampleCount > 30) ? 30 : sampleCount;
    float medianSoil = getMedian(soilSamples, validSamples);

    // 1. Gửi qua LoRa
    sendSensorData(avgT, avgH, avgP, avgLux, medianSoil);

    // 2. Luôn lưu vào SD Card
    saveToSD(avgT, avgH, avgP, avgLux, medianSoil, flowRateLMin, totalVolumeL);

    Serial.println("--- 📊 5m Summary (Soil uses Median Filter) ---");
    Serial.printf("T=%.2fC H=%.2f%% P=%.2fhPa Lux=%.2f Soil=%.2f%% Flow=%.3fL/m Vol=%.3fL\n",
                  avgT, avgH, avgP, avgLux, medianSoil, flowRateLMin, totalVolumeL);

    // Reset bộ tích lũy
    sumT = sumH = sumP = sumLux = 0;
    sampleCount = 0;
    lastSendMs  = millis();
  }

  delay(10);
}
