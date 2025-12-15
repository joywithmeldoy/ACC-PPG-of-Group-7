#include <Wire.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>

#include <SparkFun_MAX1704x_Fuel_Gauge_Arduino_Library.h>
#include <string.h>

// Threshold: accel magnitude > 0.007 g counts as motion
const float MOTION_THRESHOLD = 0.007f;  

// Debounce count (how many consecutive samples to confirm motion/still)
const int MOTION_DEBOUNCE_COUNT = 4;
unsigned long lastBattSendMs = 0;
// ================== LIS2DE12 Section ==================
#define LIS2DE12_ADDR 0x18   // If unsure, run an I2C scan first (usually 0x19 or 0x18)

// Registers
#define REG_WHO_AM_I     0x0F
#define REG_CTRL1        0x20
#define REG_CTRL2        0x21    // CTRL_REG2 — internal HPF / FDS live here
#define REG_CTRL4        0x23
#define REG_OUT_X_L      0x28

// ================== BLE Section ==================
#define BLE_DEVICE_NAME    "ESP32C3_LIS2DE12"
#define BLE_SERVICE_UUID   "12345678-1234-1234-1234-1234567890ab"
#define BLE_CHAR_UUID_ACC  "abcdef01-1234-1234-1234-1234567890ab"
// Battery characteristic UUID (keep consistent with PPG)
#define BLE_CHAR_UUID_BATT "012fffff-1234-1234-1234-1234567890ab"

BLEServer*         pServer        = nullptr;
BLEService*        pService       = nullptr;
BLECharacteristic* pCharAcc       = nullptr;
// New: battery characteristic
BLECharacteristic* pCharBattery   = nullptr;

bool deviceConnected              = false;

// ================== MAX17048 Fuel Gauge ==================
SFE_MAX1704X lipo(MAX1704X_MAX17048);
bool batteryInitOk = false;

// ------------------- Custom callbacks -------------------
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
  }
  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    // Restart advertising after disconnect (easy to reconnect)
    BLEDevice::startAdvertising();
  }
};

// ================== Initialize MAX17048 ==================
bool initBattery() {
  Serial.println("Initializing MAX17048...");

  if (lipo.begin() == false) { // Use default Wire
    Serial.println(">>> MAX17048 not found! Check wiring.");
    return false;
  }

  Serial.println("MAX17048 init done.");
  return true;
}

// ================== 2nd-order Butterworth Low-pass (fs=100Hz, fc≈5Hz) ==================
struct BiquadState {
  float x1, x2; // Previous two inputs
  float y1, y2; // Previous two outputs
};

// One filter state per axis (X/Y/Z)
BiquadState lpfX = {0,0,0,0};
BiquadState lpfY = {0,0,0,0};
BiquadState lpfZ = {0,0,0,0};

// Filter coefficients (fs=100Hz, fc≈5Hz, 2nd-order Butterworth)
const float b0 = 0.0200833656f;
const float b1 = 0.0401667311f;
const float b2 = 0.0200833656f;
const float a1 = -1.5610180758f;
const float a2 =  0.6413515381f;

// One-step 2nd-order IIR
float biquadLPF(float x, BiquadState &s) {
  float y = b0 * x + b1 * s.x1 + b2 * s.x2
                  - a1 * s.y1 - a2 * s.y2;

  // Update state
  s.x2 = s.x1;
  s.x1 = x;
  s.y2 = s.y1;
  s.y1 = y;

  return y;
}

// ================== Motion Detection & Debounce ==================
bool isMoving = false;      // Current state: moving?
int moveCount = 0;          // Consecutive "moving" count
int stillCount = 0;         // Consecutive "still" count

void updateMotion(float magnitude) {
  if (magnitude > MOTION_THRESHOLD) {
    moveCount++;
    stillCount = 0;

    if (moveCount > MOTION_DEBOUNCE_COUNT)
      isMoving = true;   // Over threshold for N samples → motion confirmed

  } else {
    stillCount++;
    moveCount = 0;

    if (stillCount > MOTION_DEBOUNCE_COUNT)
      isMoving = false;  // Under threshold for N samples → stillness confirmed
  }
}

// ================== I2C helper functions ==================
void scanI2C() {
  Serial.println("Scanning I2C...");
  for (uint8_t addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf("Found device at 0x%02X\n", addr);
    }
  }
  Serial.println("Scan done.\n");
}

void writeReg(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(LIS2DE12_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readReg(uint8_t reg) {
  Wire.beginTransmission(LIS2DE12_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LIS2DE12_ADDR, 1);
  return Wire.read();
}

// Read 6 bytes and convert to 12-bit values
void readXYZ(int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(LIS2DE12_ADDR);
  Wire.write(REG_OUT_X_L | 0x80); // auto-increment
  Wire.endTransmission(false);

  Wire.requestFrom(LIS2DE12_ADDR, 6);
  uint8_t xl = Wire.read();
  uint8_t xh = Wire.read();
  uint8_t yl = Wire.read();
  uint8_t yh = Wire.read();
  uint8_t zl = Wire.read();
  uint8_t zh = Wire.read();

  // LIS2DE12: 12-bit left-aligned in 16-bit
  x = (int16_t)(xh << 8 | xl) >> 4;
  y = (int16_t)(yh << 8 | yl) >> 4;
  z = (int16_t)(zh << 8 | zl) >> 4;
}

// ================== BLE initialization ==================
void initBLE() {
  BLEDevice::init(BLE_DEVICE_NAME);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  pService = pServer->createService(BLE_SERVICE_UUID);

  // Characteristic 1: ACC read + Notify (unchanged)
  pCharAcc = pService->createCharacteristic(
    BLE_CHAR_UUID_ACC,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharAcc->addDescriptor(new BLE2902());  // Allow the phone to enable Notify

  // Characteristic 2: Battery read + Notify (added)
  pCharBattery = pService->createCharacteristic(
    BLE_CHAR_UUID_BATT,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharBattery->addDescriptor(new BLE2902());

  // Initial UTF-8 text value so the phone can display it as a string
  const char *initBattStr = "Batt=0.000V,0.0%";
  pCharBattery->setValue((uint8_t*)initBattStr, strlen(initBattStr));

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(BLE_SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();

  Serial.println("BLE init done, advertising started.");
}

// ================== Setup ==================
void setup() {
  Serial.begin(115200);
  delay(500);

  // Start I2C
  Wire.begin(7, 6);

  scanI2C();

  uint8_t who = readReg(REG_WHO_AM_I);
  Serial.printf("WHO_AM_I = 0x%02X (expect 0x33)\n", who);

  // ---- LIS2DE12 init ----
  // CTRL1: ODR=100Hz, enable X/Y/Z
  writeReg(REG_CTRL1,
           0b01010111);
  // bit7-4: ODR=100Hz (0101)
  // bit2: Z enable
  // bit1: Y enable
  // bit0: X enable

  // CTRL2: enable internal HPF + select filtered output
  // HPM1:HPM0 = 00 (normal mode)
  // HPCF2:HPCF1 = 11 → HPF cutoff ~0.2 Hz @ 100Hz ODR
  // FDS = 1 → output registers use "filtered" data
  // HPCLICK / HP_IA2 / HP_IA1 = 0
  writeReg(REG_CTRL2, 0b00111000);   // 0x38
  Serial.println("CTRL2 set: internal HPF enabled, filtered data selected.");

  // CTRL4: high-resolution + ±2g
  writeReg(REG_CTRL4,
           0b10001000);
  // bit7: HR=1 → High-resolution 12-bit
  // bit5-4: FS=00 → ±2g

  Serial.println("LIS2DE12 init done.\n");

  // ---- MAX17048 init ----
  batteryInitOk = initBattery();
  if (!batteryInitOk) {
    Serial.println("MAX17048 init failed. Battery data will be 0.");
  }

  // ---- BLE init ----
  initBLE();
}

// ================== Loop ==================
void loop() {
  int16_t x_raw, y_raw, z_raw;
  readXYZ(x_raw, y_raw, z_raw);

  float xf_raw = x_raw * 0.001f;
  float yf_raw = y_raw * 0.001f;
  float zf_raw = z_raw * 0.001f;

  float xf = biquadLPF(xf_raw, lpfX);
  float yf = biquadLPF(yf_raw, lpfY);
  float zf = biquadLPF(zf_raw, lpfZ);

  // ===== Motion magnitude =====
  float magnitude = sqrt(xf*xf + yf*yf + zf*zf);

  // ===== Update motion (debounce) =====
  updateMotion(magnitude);

    // ===== Print =====
  Serial.printf("ACC LPF: X=%.3f  Y=%.3f  Z=%.3f | mag=%.3f | moving=%d\n",
                xf, yf, zf, magnitude, isMoving);

  // ===== BLE Notify: acceleration =====
  if (deviceConnected && pCharAcc != nullptr) {
    char buf[128];
    snprintf(buf, sizeof(buf), "%.3f,%.3f,%.3f,%s",
             xf, yf, zf, isMoving ? "Moving" : "Rest");
    pCharAcc->setValue((uint8_t*)buf, strlen(buf));
    pCharAcc->notify();
  }

  // ===== BLE Notify: battery (every 1s) =====
  if (deviceConnected && pCharBattery != nullptr) {
    unsigned long now = millis();
    if (now - lastBattSendMs > 1000) { // 1s
      float v   = 0.0f;
      float soc = 0.0f;
      if (batteryInitOk) {
        v   = lipo.getVoltage(); // Unit: V
        soc = lipo.getSOC();     // Unit: %
      }

      char battMsg[64];
      // e.g.: Batt=3.953V,80.2%
      snprintf(battMsg, sizeof(battMsg), "Batt=%.3fV,%.1f%%", v, soc);

      pCharBattery->setValue((uint8_t*)battMsg, strlen(battMsg));
      pCharBattery->notify();

      lastBattSendMs = now;
    }
  }

  delay(10);  // ~100Hz
}