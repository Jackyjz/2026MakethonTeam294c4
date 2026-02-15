#include <Arduino.h>
#include <Wire.h>
#include "MAX30100_PulseOximeter.h"
#include <DHT.h>

// BLE
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// GPS (DFR1103)
#include "DFRobot_GNSSAndRTC.h"

// I2S audio (ESP32)
#include "driver/i2s.h"
#include <math.h>

// ================== PINS ==================
// MAX30100 I2C (Bus0 = Wire)
static const int SDA0 = 23;
static const int SCL0 = 22;

// MPU6050 + GPS I2C (Bus1 = Wire1)
static const int SDA1 = 19;
static const int SCL1 = 18;
static const uint8_t MPU_ADDR = 0x68;
static const uint8_t GPS_ADDR = 0x66;

// DHT
#define DHTPIN 4
#define DHTTYPE DHT11

// I2S speaker pins (your wiring)
static const int I2S_BCLK = 26;
static const int I2S_LRC  = 25;
static const int I2S_DIN  = 27;

// BUTTON (acknowledge)
static const int BTN_ACK = 13;   // button to GND

// ================== OBJECTS ==================
PulseOximeter pox;
DHT dht(DHTPIN, DHTTYPE);
DFRobot_GNSSAndRTC_I2C gnss(&Wire1, GPS_ADDR);

// ================== BLE (ONE JSON CHARACTERISTIC) ==================
static bool bleConnected = false;
static BLECharacteristic *chJSON = nullptr;

#define BLE_SERVICE_UUID    "12345678-1234-1234-1234-1234567890ab"
#define BLE_CHAR_JSON_UUID  "12345678-1234-1234-1234-1234567890af"

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override { bleConnected = true; }
  void onDisconnect(BLEServer*) override {
    bleConnected = false;
    BLEDevice::startAdvertising();
  }
};

// ================== STATE MACHINE ==================
enum RunState { NORMAL, WARNING };
RunState g_state = NORMAL;

String g_reason = "none";

// ================== TIMERS ==================
uint32_t lastHRReport   = 0;
uint32_t lastMPUReport  = 0;
uint32_t lastDHTReport  = 0;
uint32_t lastGpsPollMs  = 0;
uint32_t lastBLEJsonTx  = 0;
uint32_t lastBeepMs     = 0;

// ================== LATEST SENSOR VALUES (live) ==================
float g_hr = NAN;
float g_ax = NAN, g_ay = NAN, g_az = NAN;
float g_gx = NAN, g_gy = NAN, g_gz = NAN;
float g_mpu_temp = NAN;
float g_t = NAN, g_h = NAN;

// ================== SNAPSHOT VALUES (frozen during WARNING) ==================
float s_hr = NAN;
float s_ax = NAN, s_ay = NAN, s_az = NAN;
float s_gx = NAN, s_gy = NAN, s_gz = NAN;
float s_mpu_temp = NAN;
float s_t = NAN, s_h = NAN;

// ================== GPS VALUES (updates only in WARNING) ==================
bool   g_gps_ok = false;
double g_lat = 0.0, g_lon = 0.0, g_alt = 0.0;
uint8_t g_sats = 0;
int     g_gnss_mode = 0;

bool gpsReady = false;

// ================== HR STABILITY / ARMING ==================
bool     hrArmed = false;          // true after stable HR window
uint32_t hrStableStartMs = 0;
float    hrEma = NAN;              // smoothed HR
int      hrBadCount = 0;           // consecutive abnormal samples

// ================== BEAT GATING ==================
volatile uint32_t lastBeatMsISR = 0;

void onBeatDetected() {
  lastBeatMsISR = millis();
}

// ================== MPU6050 HELPERS (Wire1) ==================
static inline int16_t readI16BE(const uint8_t *p) {
  return (int16_t)((p[0] << 8) | p[1]);
}

bool mpuWriteByte(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  return (Wire1.endTransmission() == 0);
}

bool mpuReadBytes(uint8_t startReg, uint8_t *buf, size_t len) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(startReg);
  if (Wire1.endTransmission(false) != 0) return false;
  size_t got = Wire1.requestFrom(MPU_ADDR, (uint8_t)len, (uint8_t)true);
  if (got != len) return false;
  for (size_t i = 0; i < len; i++) buf[i] = Wire1.read();
  return true;
}

bool mpuInit() {
  if (!mpuWriteByte(0x6B, 0x00)) return false; // Wake up
  if (!mpuWriteByte(0x1C, 0x00)) return false; // Accel ±2g
  if (!mpuWriteByte(0x1B, 0x00)) return false; // Gyro ±250 dps
  if (!mpuWriteByte(0x1A, 0x03)) return false; // DLPF
  return true;
}

// ================== I2S AUDIO (short beep) ==================
static bool i2sReady = false;

void i2sInit() {
  i2s_config_t cfg;
  memset(&cfg, 0, sizeof(cfg));
  cfg.mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX);
  cfg.sample_rate = 16000;
  cfg.bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT;
  cfg.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT; // mono
  cfg.communication_format = I2S_COMM_FORMAT_I2S_MSB;
  cfg.intr_alloc_flags = 0;
  cfg.dma_buf_count = 4;
  cfg.dma_buf_len = 256;
  cfg.use_apll = false;
  cfg.tx_desc_auto_clear = true;

  i2s_pin_config_t pins;
  pins.bck_io_num = I2S_BCLK;
  pins.ws_io_num = I2S_LRC;
  pins.data_out_num = I2S_DIN;
  pins.data_in_num = I2S_PIN_NO_CHANGE;

  i2s_driver_install(I2S_NUM_0, &cfg, 0, nullptr);
  i2s_set_pin(I2S_NUM_0, &pins);
  i2s_zero_dma_buffer(I2S_NUM_0);
  i2sReady = true;
}

void playBeepOnce(int freqHz = 1400, float volume = 0.06f, int ms = 20) {
  if (!i2sReady) return;

  const int sampleRate = 16000;
  int totalSamples = (sampleRate * ms) / 1000;
  static int16_t buf[256];

  int generated = 0;
  while (generated < totalSamples) {
    int chunk = min(256, totalSamples - generated);
    for (int i = 0; i < chunk; i++) {
      float t = (float)(generated + i) / (float)sampleRate;
      float s = sinf(2.0f * 3.1415926f * freqHz * t);
      buf[i] = (int16_t)(s * (32767.0f * volume));
    }
    size_t bytesWritten = 0;
    i2s_write(I2S_NUM_0, buf, chunk * sizeof(int16_t), &bytesWritten, 0);
    generated += chunk;
  }
}

void warningBeepService() {
  if (g_state != WARNING) return;
  uint32_t now = millis();
  if (now - lastBeepMs >= 1000) {
    playBeepOnce(1400, 0.06f, 20);
    lastBeepMs = now;
  }
}

// ================== WARNING THRESHOLDS ==================
static inline float accelMagG(float ax, float ay, float az) {
  return sqrtf(ax*ax + ay*ay + az*az);
}

bool shouldTriggerWarning(String &reasonOut) {
  // Impact threshold
  bool accel_valid = (!isnan(g_ax) && !isnan(g_ay) && !isnan(g_az));
  float amag = accel_valid ? accelMagG(g_ax, g_ay, g_az) : 0.0f;
  bool impact = (accel_valid && amag > 2.2f);

  // HR threshold ONLY if ARMED
  bool hr_bad = false;
  if (hrArmed && !isnan(g_hr)) {
    if (g_hr < 45.0f || g_hr > 160.0f) hrBadCount++;
    else hrBadCount = 0;
    hr_bad = (hrBadCount >= 3);   // require 3 bad seconds
  } else {
    hrBadCount = 0;
  }

  // Temp threshold (environment)
  bool temp_valid = (!isnan(g_t));
  bool temp_hot = (temp_valid && g_t > 40.0f);

  if (impact) { reasonOut = "impact"; return true; }
  if (hr_bad) { reasonOut = "hr"; return true; }
  if (temp_hot) { reasonOut = "temp"; return true; }

  reasonOut = "none";
  return false;
}

void snapshotSensors() {
  s_hr = g_hr;

  s_ax = g_ax; s_ay = g_ay; s_az = g_az;
  s_gx = g_gx; s_gy = g_gy; s_gz = g_gz;
  s_mpu_temp = g_mpu_temp;

  s_t = g_t; s_h = g_h;
}

// ================== GPS (only in WARNING) ==================
void gpsStartIfNeeded() {
  if (gpsReady) return;

  if (!gnss.begin()) {
    Serial.println("[GPS] begin failed on Wire1. Check DFR1103 wiring to SDA=19 SCL=18, addr 0x66.");
    gpsReady = false;
    return;
  }
  gnss.enablePower();
  gpsReady = true;
  Serial.println("[GPS] started on Wire1.");
}

void gpsPoll10s() {
  if (g_state != WARNING) return;

  gpsStartIfNeeded();
  if (!gpsReady) return;

  uint32_t now = millis();
  if (now - lastGpsPollMs < 10000) return; // every 10 seconds
  lastGpsPollMs = now;

  g_sats = gnss.getNumSatUsed();
  g_gnss_mode = gnss.getGnssMode();

  if (g_sats >= 4) {
    auto lat = gnss.getLat();
    auto lon = gnss.getLon();
    g_lat = lat.latitudeDegree;
    g_lon = lon.lonitudeDegree;
    g_alt = gnss.getAlt();
    g_gps_ok = true;
  } else {
    g_gps_ok = false;
  }

  Serial.printf("[GPS] mode=%d sats=%u ok=%d lat=%.6f lon=%.6f\n",
                g_gnss_mode, g_sats, (int)g_gps_ok, g_lat, g_lon);
}

// ================== BLE JSON ==================
static void bleSendJsonLine() {
  if (!bleConnected || !chJSON) return;

  float hr = (g_state == WARNING) ? s_hr : g_hr;
  float ax = (g_state == WARNING) ? s_ax : g_ax;
  float ay = (g_state == WARNING) ? s_ay : g_ay;
  float az = (g_state == WARNING) ? s_az : g_az;
  float t  = (g_state == WARNING) ? s_t  : g_t;
  float h  = (g_state == WARNING) ? s_h  : g_h;

  char msg[200];

  if (g_state == WARNING) {
    snprintf(msg, sizeof(msg),
      "{\"st\":\"W\",\"r\":\"%s\",\"hr\":%.1f,\"a\":[%.2f,%.2f,%.2f],\"t\":%.1f,\"h\":%.1f,"
      "\"g\":{\"ok\":%d,\"sa\":%u,\"la\":%.5f,\"lo\":%.5f}}\n",
      g_reason.c_str(),
      isnan(hr)?-1.0f:hr,
      isnan(ax)?0.0f:ax, isnan(ay)?0.0f:ay, isnan(az)?0.0f:az,
      isnan(t)?-1000.0f:t,
      isnan(h)?-1.0f:h,
      (int)g_gps_ok, g_sats, g_lat, g_lon
    );
  } else {
    snprintf(msg, sizeof(msg),
      "{\"st\":\"N\",\"r\":\"none\",\"hr\":%.1f,\"a\":[%.2f,%.2f,%.2f],\"t\":%.1f,\"h\":%.1f}\n",
      isnan(hr)?-1.0f:hr,
      isnan(ax)?0.0f:ax, isnan(ay)?0.0f:ay, isnan(az)?0.0f:az,
      isnan(t)?-1000.0f:t,
      isnan(h)?-1.0f:h
    );
  }

  chJSON->setValue((uint8_t*)msg, strlen(msg));
  chJSON->notify();
}

void bleJsonService10s() {
  uint32_t now = millis();
  if (now - lastBLEJsonTx < 10000) return;
  lastBLEJsonTx = now;
  bleSendJsonLine();
}

// ================== SENSOR RESET / REINIT ==================
void resetAndReinitSensors() {
  Serial.println("[RESET] Re-initializing sensors...");

  // Stop warning extras
  gpsReady = false;
  g_gps_ok = false;
  g_lat = g_lon = g_alt = 0.0;
  g_sats = 0;
  g_gnss_mode = 0;

  // Restart I2C buses (helps recover stuck I2C)
  Wire.end();
  Wire1.end();
  delay(30);

  Wire.begin(SDA0, SCL0, 100000);
  Wire.setTimeOut(50);

  Wire1.begin(SDA1, SCL1, 100000);
  Wire1.setTimeOut(50);

  // MPU re-init
  if (!mpuInit()) Serial.println("[RESET] MPU6050 init failed!");
  else Serial.println("[RESET] MPU6050 OK.");

  // MAX30100 re-init
  if (!pox.begin()) Serial.println("[RESET] MAX30100 init failed!");
  else {
    pox.setOnBeatDetectedCallback(onBeatDetected);
    Serial.println("[RESET] MAX30100 OK.");
  }

  // DHT re-init
  dht.begin();
  Serial.println("[RESET] DHT OK.");

  // Clear live values
  g_hr = NAN;
  g_ax = g_ay = g_az = NAN;
  g_gx = g_gy = g_gz = NAN;
  g_mpu_temp = NAN;
  g_t = g_h = NAN;

  // Clear HR stability state
  hrArmed = false;
  hrStableStartMs = 0;
  hrEma = NAN;
  hrBadCount = 0;
  lastBeatMsISR = 0;

  Serial.println("[RESET] Done.");
}

// ================== BUTTON (press once) ==================
bool ackButtonPressedOnce() {
  static bool wasPressed = false;
  static uint32_t t0 = 0;

  int v = digitalRead(BTN_ACK); // INPUT_PULLUP: LOW = pressed

  // pressed
  if (!wasPressed && v == LOW) {
    wasPressed = true;
    t0 = millis();
  }

  // released after at least 30ms (debounce)
  if (wasPressed && v == HIGH) {
    if (millis() - t0 >= 30) {
      wasPressed = false;
      return true;   // one full press cycle
    }
    // if it released too fast, treat as bounce
    wasPressed = false;
  }

  return false;
}

// ================== SETUP ==================
void setup() {
  Serial.begin(115200);
  delay(200);

  pinMode(BTN_ACK, INPUT_PULLUP);

  i2sInit();
  Serial.println("I2S speaker ready.");

  // BLE
  BLEDevice::init("ESP32-Sensors");
  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(BLE_SERVICE_UUID);
  chJSON = service->createCharacteristic(
    BLE_CHAR_JSON_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  chJSON->addDescriptor(new BLE2902());
  service->start();

  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(BLE_SERVICE_UUID);
  adv->start();
  Serial.println("BLE advertising: ESP32-Sensors (JSON notify)");

  // I2C buses
  Wire.begin(SDA0, SCL0, 100000);
  Wire.setTimeOut(50);

  Wire1.begin(SDA1, SCL1, 100000);
  Wire1.setTimeOut(50);

  // Init MPU
  Serial.println("Init MPU6050...");
  if (!mpuInit()) {
    Serial.println("ERROR: MPU6050 init failed.");
    while (true) delay(1000);
  }
  Serial.println("MPU6050 OK.");

  // Init MAX30100
  Serial.println("Init MAX30100...");
  if (!pox.begin()) {
    Serial.println("ERROR: MAX30100 init failed.");
    while (true) delay(1000);
  }
  pox.setOnBeatDetectedCallback(onBeatDetected);
  Serial.println("MAX30100 OK.");

  // Init DHT
  Serial.println("Init DHT...");
  dht.begin();
  Serial.println("DHT OK.");

  Serial.println("Running...");
}

// ================== LOOP ==================
void loop() {
  uint32_t now = millis();

  // In NORMAL mode, keep MAX30100 serviced very frequently.
  if (g_state == NORMAL) {
    pox.update();
  }

  // ================= NORMAL: read sensors =================
  if (g_state == NORMAL) {

    // HR every 1s + stability/arming
    if (now - lastHRReport >= 1000) {
      lastHRReport = now;

      float hrRaw = pox.getHeartRate();
      uint32_t lastBeat = lastBeatMsISR;
      bool fingerLikely = (lastBeat > 0) && ((now - lastBeat) < 3000);

      if (!fingerLikely) {
        g_hr = NAN;
        hrArmed = false;
        hrStableStartMs = 0;
        hrBadCount = 0;
        hrEma = NAN;
        Serial.println("[MAX30100] NOFINGER");
      } else {
        bool hrInRange = (hrRaw >= 40.0f && hrRaw <= 180.0f);

        if (!hrInRange) {
          g_hr = NAN;
          hrArmed = false;
          hrStableStartMs = 0;
          hrBadCount = 0;
          Serial.printf("[MAX30100] HR: -- (unstable) raw=%.1f\n", hrRaw);
        } else {
          if (isnan(hrEma)) hrEma = hrRaw;
          hrEma = 0.8f * hrEma + 0.2f * hrRaw;
          g_hr = hrEma;

          float err = fabsf(hrRaw - hrEma);
          bool stableNow = (err < 5.0f); // within 5 bpm of EMA

          if (stableNow) {
            if (hrStableStartMs == 0) hrStableStartMs = now;
            if ((now - hrStableStartMs) >= 8000) hrArmed = true; // armed after 8s stable
          } else {
            hrStableStartMs = 0;
            hrArmed = false;
          }

          Serial.printf("[MAX30100] HR: %.1f bpm %s\n", g_hr, hrArmed ? "(ARMED)" : "(warming)");
        }
      }
    }

    // MPU every 500ms
    if (now - lastMPUReport >= 500) {
      lastMPUReport = now;
      uint8_t buf[14];

      if (mpuReadBytes(0x3B, buf, sizeof(buf))) {
        int16_t ax_raw = readI16BE(&buf[0]);
        int16_t ay_raw = readI16BE(&buf[2]);
        int16_t az_raw = readI16BE(&buf[4]);
        int16_t t_raw  = readI16BE(&buf[6]);
        int16_t gx_raw = readI16BE(&buf[8]);
        int16_t gy_raw = readI16BE(&buf[10]);
        int16_t gz_raw = readI16BE(&buf[12]);

        g_ax = ax_raw / 16384.0f;
        g_ay = ay_raw / 16384.0f;
        g_az = az_raw / 16384.0f;

        g_gx = gx_raw / 131.0f;
        g_gy = gy_raw / 131.0f;
        g_gz = gz_raw / 131.0f;

        g_mpu_temp = (t_raw / 340.0f) + 36.53f;

        Serial.printf("[MPU6050] A=(%.2f,%.2f,%.2f) G=(%.1f,%.1f,%.1f) T=%.2fC\n",
                      g_ax, g_ay, g_az, g_gx, g_gy, g_gz, g_mpu_temp);
      } else {
        Serial.println("[MPU6050] read failed");
      }
    }

    // DHT every 2s
    if (now - lastDHTReport >= 2000) {
      lastDHTReport = now;
      float h = dht.readHumidity();
      float t = dht.readTemperature();

      if (isnan(h) || isnan(t)) {
        Serial.println("[DHT] read failed");
        g_h = NAN; g_t = NAN;
      } else {
        g_h = h; g_t = t;
        Serial.printf("[DHT] T=%.1fC H=%.1f%%\n", g_t, g_h);
      }
    }

    // Check warning trigger
    String reason;
    if (shouldTriggerWarning(reason)) {
      g_state = WARNING;
      g_reason = reason;

      snapshotSensors();     // freeze snapshot
      lastGpsPollMs = 0;     // allow immediate GPS poll
      lastBLEJsonTx = 0;     // allow immediate JSON
      lastBeepMs = 0;

      bleSendJsonLine(); // send JSON immediately
      lastBLEJsonTx = millis();

      Serial.printf("[WARNING] Triggered reason=%s. Beeping until button press. GPS/JSON every 10s.\n",
                    g_reason.c_str());
    }

    // JSON every 10s in NORMAL
    bleJsonService10s();
  }

  // ================= WARNING: beep until button press; only GPS+JSON =================
  if (g_state == WARNING) {
    warningBeepService();
    gpsPoll10s();          // GPS poll every 10s
    bleJsonService10s();   // JSON every 10s (snapshot + gps)

    if (ackButtonPressedOnce()) {
      Serial.println("[WARNING] ACK button pressed -> reset sensors and resume.");
      resetAndReinitSensors();

      g_state = NORMAL;
      g_reason = "none";

      // immediately send JSON
      bleSendJsonLine();
      lastBLEJsonTx = millis();

      // refresh timers
      lastHRReport = lastMPUReport = lastDHTReport = 0;
      lastBLEJsonTx = 0;
      lastGpsPollMs = 0;
      lastBeepMs = 0;
    }
  }

  delay(1);
}
