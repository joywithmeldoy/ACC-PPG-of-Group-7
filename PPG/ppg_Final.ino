#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <string.h>

// MAX17048 fuel gauge object
SFE_MAX1704X lipo(MAX1704X_MAX17048);

// ===================== Configuration =====================

// I2C pins
#define SDA_PIN 7
#define SCL_PIN 6

// Sampling & algorithm parameters
#define BUFFER_LEN 50           // SpO2 algorithm uses 100 samples per window
const uint16_t SAMPLE_RATE = 50; // Hz; keep consistent with sampleRate in setup()

// BLE UUID
#define BLE_SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define BLE_CHARACTERISTIC_UUID "abcdef01-1234-1234-1234-1234567890ab"
#define BLE_CHARACTERISTIC_UUID_BATT "012fffff-1234-1234-1234-1234567890ab"

// ===================== Globals =====================

MAX30105 particleSensor;

uint32_t irBuffer[BUFFER_LEN];
uint32_t redBuffer[BUFFER_LEN];

int32_t spo2;
int8_t  validSPO2;
int32_t heartRate;
int8_t  validHeartRate;

BLEServer*        pServer          = nullptr;
BLECharacteristic* pCharHrSpo2     = nullptr;
bool deviceConnected              = false;
bool oldDeviceConnected           = false;

// Added: battery characteristic
BLECharacteristic* pCharBattery   = nullptr;
// Added: track whether the fuel gauge initialized successfully
bool batteryInitOk                = false;

// Custom callbacks: mark connect/disconnect state
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
  }
};

// ===================== Init MAX30102 =====================

bool initMAX30102() {
  Serial.println("Initializing MAX30102...");

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println(">>> MAX30102 not found! Check wiring (SDA/SCL/VCC/GND).");
    return false;
  }

  byte ledBrightness = 0x6F; // LED current
  byte sampleAverage = 4;    // sample averaging
  byte ledMode       = 2;    // 2 = RED + IR
  byte sampleRate    = 50;   // 50 Hz
  int  pulseWidth    = 411;  // max resolution
  int  adcRange      = 16384;

  particleSensor.setup(ledBrightness, sampleAverage, ledMode,
                       sampleRate, pulseWidth, adcRange);

  particleSensor.setPulseAmplitudeRed(0x7F);
  particleSensor.setPulseAmplitudeIR(0x7F);
  particleSensor.setPulseAmplitudeGreen(0);

  Serial.println("MAX30102 init done. Put your finger on the sensor.");
  return true;
}

// ===================== Init MAX17048 =====================

bool initBattery() {
  Serial.println("Initializing MAX17048...");


  if (lipo.begin() == false) { // use default Wire
    Serial.println(">>> MAX17048 not found! Check wiring.");
    return false;
  }

  Serial.println("MAX17048 init done.");
  return true;
}

// ===================== Init BLE =====================

void initBLE() {
  Serial.println("Initializing BLE...");

  BLEDevice::init("ESP32C3_PPG");  // Device name shown during phone scan

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService* pService = pServer->createService(BLE_SERVICE_UUID);

  // Create a characteristic to send HR / SpO2
  // Payload: 4 bytes
  // [0-1] HR (uint16, little-endian)
  // [2]   SpO2 (uint8)
  // [3]   flags bit0: HR valid, bit1: SpO2 valid
  pCharHrSpo2 = pService->createCharacteristic(
                    BLE_CHARACTERISTIC_UUID,
                    BLECharacteristic::PROPERTY_READ   |
                    BLECharacteristic::PROPERTY_NOTIFY
                  );

  // Added: create a characteristic to send battery voltage & SoC
  // Payload: 8 bytes (2 floats, little-endian IEEE754)
  // [0-3] Voltage (float, unit: V)
  // [4-7] State of charge (float, unit: %)
  pCharBattery = pService->createCharacteristic(
                    BLE_CHARACTERISTIC_UUID_BATT,
                    BLECharacteristic::PROPERTY_READ |
                    BLECharacteristic::PROPERTY_NOTIFY
                 );

  // Initial UTF-8 text value (easy to show as a string on the phone)
  const char *initBattStr = "Batt=0.000V,0.0%";
  pCharBattery->setValue((uint8_t*)initBattStr, strlen(initBattStr));

  const char *initHrStr = "HR=-1,SpO2=-1";
  pCharHrSpo2->setValue((uint8_t*)initHrStr, strlen(initHrStr));

  pService->start();

  // Start advertising
  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  BLEDevice::startAdvertising();

  Serial.println("BLE advertising started. You can connect with phone now.");
}

// ===================== Pack HR/SpO2 and send via BLE =====================

void sendHrSpo2OverBLE() {
  if (!deviceConnected) return;

  // ========= Send HR / SpO2 (UTF-8 text) =========
  if (pCharHrSpo2 != nullptr) {
    int hr_show   = (validHeartRate && heartRate > 0 && heartRate < 250) ? heartRate : -1;
    int spo2_show = (validSPO2 && spo2 > 0 && spo2 <= 100) ? spo2 : -1;

    char msg[64];
    // Example: HR=75,SpO2=98 (UTF-8 text)
    snprintf(msg, sizeof(msg), "HR=%d,SpO2=%d", hr_show, spo2_show);

    // We send a standard C string -> UTF-8 bytes
    pCharHrSpo2->setValue((uint8_t*)msg, strlen(msg));
    pCharHrSpo2->notify();
  }

  // ========= Send battery voltage & SoC (UTF-8 text) =========
  if (pCharBattery != nullptr) {
    float v   = 0.0f;
    float soc = 0.0f;
    if (batteryInitOk) {
      v   = lipo.getVoltage(); // unit: V
      soc = lipo.getSOC();     // unit: %
    }

    char battMsg[64];
    // Example: Batt=3.953V,80.2%
    //snprintf(battMsg, sizeof(battMsg), "Batt=%.3fV,%.1f%%", v, soc);
    snprintf(battMsg, sizeof(battMsg), "Batt=%.3fV%", v);

    pCharBattery->setValue((uint8_t*)battMsg, strlen(battMsg));
    pCharBattery->notify();
  }
}

// ===================== Dynamic-threshold HR detector =====================
// Use IR channel with dynamic-threshold peak detection; update global heartRate / validHeartRate
void computeHR_fixedThreshold(uint32_t *ir, int len) {
  // 1) remove DC component
  double sum = 0;
  for (int i = 0; i < len; i++) sum += ir[i];
  float mean = sum / len;

  // 2) compute standard deviation (std) to set a threshold
  double var = 0;
  for (int i = 0; i < len; i++) {
    float x = ir[i] - mean;
    var += x * x;
  }
  float std = sqrt(var / len);

  // threshold = 0.7 * std; tune up/down as needed
  float thr = 0.7f * std;

  // 3) peak detect: a rising-edge crossing thr counts as a beat
  int peakIndex[BUFFER_LEN];
  int peakCount = 0;

  bool above = false;
  for (int n = 1; n < len; n++) {
    float prev = ir[n - 1] - mean;
    float curr = ir[n] - mean;

    // rising edge crosses threshold
    if (!above && prev < thr && curr >= thr) {
      if (peakCount < BUFFER_LEN) {
        peakIndex[peakCount++] = n;
      }
      above = true;
    }

    // re-arm after the signal falls back down
    if (curr < thr * 0.5f) {  
      above = false;
    }
  }

  // 4) estimate HR from peak-to-peak intervals
  if (peakCount < 2) {
    return;
  }

  float sumDt = 0;
  int   cnt   = 0;

  for (int i = 1; i < peakCount; i++) {
    int ds = peakIndex[i] - peakIndex[i - 1]; // samples
    float dt = ds / (float)SAMPLE_RATE;       // seconds

    // Heart rate should be 30–200 bpm → sanity-check dt
    if (dt > 0.3f && dt < 2.0f) {
      sumDt += dt;
      cnt++;
    }
  }

  if (cnt == 0) {
    return;
  }

  float avgDt = sumDt / cnt;
  float hr = 60.0f / avgDt;

  if (hr > 30 && hr < 220) {
    heartRate = (int)(hr + 0.5f);
    validHeartRate = 1;
  } 
}

// ===================== setup & loop =====================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println();
  Serial.println("===== ESP32-C3 MAX30102 + BLE HR/SpO2 =====");

  bool ok = initMAX30102();
  batteryInitOk = initBattery();   // Added: initialize MAX17048
  initBLE();

  if (!ok) {
    Serial.println("MAX30102 init failed. Will still run BLE but no data.");
  }

  if (!batteryInitOk) {
    Serial.println("MAX17048 init failed. Battery data will be 0.");
  }
}

unsigned long lastBleSendMs = 0;

void loop() {
  // 1. Handle BLE connection changes (restart advertising after disconnect)
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE device connected.");
  }
  if (!deviceConnected && oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("BLE device disconnected. Restart advertising...");
    delay(500);
    BLEDevice::startAdvertising();
  }

  // 2. If the sensor did not init properly, keep BLE running without sampling
  if (!particleSensor.safeCheck(10)) {
    // No new sensor data / not running; print a reminder once per second
    static unsigned long lastMsg = 0;
    if (millis() - lastMsg > 1000) {
      Serial.println("Waiting for MAX30102 data or sensor not found...");
      lastMsg = millis();
    }
    delay(50);
    return;
  }

  // 3. Capture one frame (BUFFER_LEN samples)
  for (int i = 0; i < BUFFER_LEN; i++) {
    while (!particleSensor.available()) {
      particleSensor.check();
    }
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i]  = particleSensor.getIR();
    particleSensor.nextSample();

    // For Arduino Serial Plotter
    Serial.print(irBuffer[i]);
    Serial.print(",");
    Serial.println(redBuffer[i]);
  }

  // 4. Run Maxim algorithm for SpO2 first (it also outputs HR; we replace HR with our own later)
  maxim_heart_rate_and_oxygen_saturation(
    irBuffer, BUFFER_LEN,
    redBuffer,
    &spo2, &validSPO2,
    &heartRate, &validHeartRate
  );
  computeHR_fixedThreshold(irBuffer, BUFFER_LEN);
  // 4b. Recompute HR using dynamic-threshold peak detection
  //computeHR_dynamic(irBuffer, BUFFER_LEN);
  // Serial print for debugging
  Serial.print("HR = ");
  if (validHeartRate) Serial.print(heartRate);
  else Serial.print("invalid");

  Serial.print(" bpm, SpO2 = ");
  if (validSPO2) Serial.print(spo2);
  else Serial.print("invalid");
  Serial.println(" %");

  // 5. Send via BLE once per second
  unsigned long now = millis();
  if (now - lastBleSendMs > 1000) {
    sendHrSpo2OverBLE();
    lastBleSendMs = now;
  }

  delay(10);
}
