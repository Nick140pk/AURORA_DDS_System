#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>
#include <SD.h>
#include <math.h>

// ============================================================================
// DDS Backup Flight Computer (Non-blocking Pyro + Non-blocking LoRa + DATA:)
// Compatible with your LoRa.h receiver settings
// ============================================================================

// ================== Pins (from your spec) ==================
constexpr uint8_t PIN_DROGUE_1 = 5;
constexpr uint8_t PIN_DROGUE_2 = 6;
constexpr uint8_t PIN_MAIN_1   = 9;
constexpr uint8_t PIN_MAIN_2   = 10;
constexpr uint8_t PIN_BUZZER   = 13;

// ================== SD (Adalogger FeatherWing) ==================
constexpr uint8_t SD_CS = 12;
File logFile;

// ================== Behavior toggles ==================
constexpr bool TEST_MODE = false;
constexpr bool DDS_LORA_TX_ENABLED = true;

// ================== LoRa params (MUST match receiver LoRa.h) ==================
constexpr float   RF95_FREQ_MHZ = 914.0f;
constexpr int8_t  RF95_TX_POWER_DBM = 20;   // 2..23 (RFM95W usually OK 20; 23 can brownout)

// ================== Flight params ==================
constexpr float     MAIN_ALT_M               = 200.0f;   // m AGL (after launch baseline)
constexpr float     MAIN_ARM_MARGIN_M        = 20.0f;    // must exceed MAIN_ALT + margin before MAIN can trigger

constexpr uint32_t  DROGUE_BACKUP_MS         = 15000;    // 15 s
constexpr uint32_t  MAIN_BACKUP_MS           = 25000;    // 25 s (backup main AFTER drogue)
constexpr uint32_t  POWER_STAB_MS            = 2000;
constexpr uint32_t  FLIGHT_LOCKOUT_MS        = 10000;    // no deploy window post-launch

constexpr float     APOGEE_NEG_VZ_THRESH     = -1.5f;    // m/s
constexpr uint32_t  APOGEE_NEG_VZ_DWELL_MS   = 250;      // ms
constexpr float     ASCENT_POS_VZ_THRESH     = +2.0f;    // m/s
constexpr uint32_t  ASCENT_POS_VZ_DWELL_MS   = 250;      // ms

constexpr float     MAX_FALL_SPEED_FOR_DROGUE = -60.0f;  // m/s
constexpr uint32_t  DROGUE_INEFFECTIVE_MS    = 400;      // ms

// ================== Pyro (NON-BLOCKING) ==================
constexpr uint16_t  PYRO_PULSE_MS            = 300;      // ms each channel ON
constexpr uint16_t  PYRO_GAP_MS              = 2000;     // gap between A and B

// ================== Launch detect using BOTH sensors ==================
constexpr uint32_t  BASELINE_MIN_MS          = 800;
constexpr float     LAUNCH_DAX_HG_G_THRESH   = 1.2f;    // ADXL +X delta (g)
constexpr float     LAUNCH_DAX_LG_G_THRESH   = 0.6f;    // BNO  +X delta (g)
constexpr uint32_t  LAUNCH_DWELL_MS          = 200;     // ms

// Launch detected beeping (NON-BLOCKING)
constexpr uint16_t  LAUNCH_BEEP_SECONDS      = 2;
constexpr uint16_t  BEEP_ON_MS               = 50;
constexpr uint16_t  BEEP_OFF_MS              = 50;

// ================== State machine ==================
enum class FlightState : uint8_t {
  BOOT, IDLE, ASCENT_LOCKOUT, ASCENT, APOGEE_DETECT,
  DROGUE_DEPLOYED, MAIN_DEPLOYED, LANDED, FAULT
};
FlightState state = FlightState::BOOT;

// ================== Sensors ==================
Adafruit_BMP3XX bmp;
Adafruit_BNO08x bno08x;
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);

#define BMP3XX_I2C_ADDR 0x77
constexpr float SEA_LEVEL_HPA = 1013.25f;
sh2_SensorValue_t bnoValue;

// ================== Radio (RadioHead RH_RF95) ==================
constexpr uint8_t RFM95_CS  = 8;
constexpr uint8_t RFM95_IRQ = 3;
constexpr uint8_t RFM95_RST = 4;
RH_RF95 rf95(RFM95_CS, RFM95_IRQ);

// ================== Globals ==================
uint32_t t_boot = 0, t_launch = 0;
uint32_t t_drogue_fire = 0, t_main_fire = 0;
bool drogueFired = false, mainFired = false;
bool sensorsOK = false;

float alt_m = 0, altRef_m = 0, vz_mps = 0;

// --- NEW: launch AGL baseline + main arming ---
float launchAlt_m = 0.0f;     // altitude at launch (in same units as alt_m)
float altAGL_m = 0.0f;        // altitude above launch baseline
float maxAltAGL_m = 0.0f;     // max AGL observed since launch
bool  mainArmed = false;      // only true after exceeding MAIN threshold + margin

float hgAx_mps2 = 0, hgAy_mps2 = 0, hgAz_mps2 = 0; // ADXL375
float lgAx_mps2 = 0, lgAy_mps2 = 0, lgAz_mps2 = 0; // BNO accel
float gyroX_dps = 0, gyroY_dps = 0, gyroZ_dps = 0;
float roll_deg = 0, pitch_deg = 0, yaw_deg = 0;
float magX_uT = 0, magY_uT = 0, magZ_uT = 0;

float am_g = 0;
float batt_V = 0;

// GPS placeholders (keep dashboard compatible)
double lat = 0.0, lon = 0.0;
float gpsAlt_m = 0.0f;
float gpsSpeed_kmph = 0.0f;

// ================== Vz estimate ==================
const int VZ_WIN = 8;
struct Sample { uint32_t t; float alt; };
Sample ring[VZ_WIN]; int ringHead = 0; int ringCount = 0;

// ================== Helpers ==================
static inline float gFromMps2(float a) { return a / 9.80665f; }
static inline bool finitef_safe(float v) { return !(isnan(v) || isinf(v)); }
static inline float safe0(float v) { return finitef_safe(v) ? v : 0.0f; }
static inline float wrap360f(float a) {
  while (a < 0) a += 360.0f;
  while (a >= 360.0f) a -= 360.0f;
  return a;
}

// ================== Boot beeps (blocking OK during setup) ==================
static void beepFast2_blocking() {
  for (int i = 0; i < 2; i++) {
    digitalWrite(PIN_BUZZER, HIGH); delay(80);
    digitalWrite(PIN_BUZZER, LOW);  delay(80);
  }
}
static void beepLong_blocking() {
  digitalWrite(PIN_BUZZER, HIGH); delay(600);
  digitalWrite(PIN_BUZZER, LOW);
}

// ================== Non-blocking buzzer ==================
enum class BuzzerMode : uint8_t { IDLE, RAPID };
static BuzzerMode buzMode = BuzzerMode::IDLE;
static bool buzState = false;
static uint32_t buzNextToggle = 0;
static uint32_t buzStopAt = 0;

static void startRapidBeep(uint16_t seconds) {
  uint32_t now = millis();
  buzMode = BuzzerMode::RAPID;
  buzState = true;
  digitalWrite(PIN_BUZZER, HIGH);
  buzNextToggle = now + BEEP_ON_MS;
  buzStopAt = now + (uint32_t)seconds * 1000UL;
}
static void stopBeep() {
  buzMode = BuzzerMode::IDLE;
  buzState = false;
  digitalWrite(PIN_BUZZER, LOW);
}
static void serviceBuzzer(uint32_t now) {
  if (buzMode == BuzzerMode::IDLE) return;
  if ((int32_t)(now - buzStopAt) >= 0) { stopBeep(); return; }
  if ((int32_t)(now - buzNextToggle) >= 0) {
    buzState = !buzState;
    digitalWrite(PIN_BUZZER, buzState ? HIGH : LOW);
    buzNextToggle = now + (buzState ? BEEP_ON_MS : BEEP_OFF_MS);
  }
}

// ================== Non-blocking pyro sequencer ==================
enum : uint8_t { PYRO_NONE=0, PYRO_DROGUE=1, PYRO_MAIN=2 };

struct PyroSequencer {
  uint8_t kind = PYRO_NONE;
  uint8_t pinA = 255;
  uint8_t pinB = 255;
  uint8_t phase = 0;   // 0 idle, 1 firing A, 2 gap, 3 firing B
  uint32_t t0 = 0;
};
static PyroSequencer pyro;

static inline bool pyroActive() { return pyro.kind != PYRO_NONE; }

// NOTE: uint8_t kind avoids Arduino auto-prototype enum-order issues
static void startPyroSequence(uint8_t kind, uint8_t a, uint8_t b) {
  if (pyroActive()) return;

  pyro.kind = kind;
  pyro.pinA = a;
  pyro.pinB = b;
  pyro.phase = 1;
  pyro.t0 = millis();

  digitalWrite(pyro.pinA, LOW);
  digitalWrite(pyro.pinB, LOW);
  digitalWrite(pyro.pinA, HIGH); // fire A
}

static void servicePyro(uint32_t now) {
  if (!pyroActive()) return;

  switch (pyro.phase) {
    case 1:
      if (now - pyro.t0 >= PYRO_PULSE_MS) {
        digitalWrite(pyro.pinA, LOW);
        pyro.phase = 2;
        pyro.t0 = now;
      }
      break;

    case 2:
      if (now - pyro.t0 >= PYRO_GAP_MS) {
        digitalWrite(pyro.pinB, HIGH);
        pyro.phase = 3;
        pyro.t0 = now;
      }
      break;

    case 3:
      if (now - pyro.t0 >= PYRO_PULSE_MS) {
        digitalWrite(pyro.pinB, LOW);
        pyro.kind = PYRO_NONE;
        pyro.phase = 0;
      }
      break;

    default:
      digitalWrite(pyro.pinA, LOW);
      digitalWrite(pyro.pinB, LOW);
      pyro.kind = PYRO_NONE;
      pyro.phase = 0;
      break;
  }
}

// ================== Non-blocking LoRa TX queue (1-deep) ==================
static bool loraTxBusy = false;
static bool loraPending = false;
static char loraPendingBuf[360];
static uint16_t loraPendingLen = 0;

static void queueLoRaTx(const char* msg) {
  if (!DDS_LORA_TX_ENABLED || !msg) return;

  uint16_t len = (uint16_t)strlen(msg);
  if (len == 0) return;
  if (len >= sizeof(loraPendingBuf)) len = sizeof(loraPendingBuf) - 1;

  if (!loraTxBusy && !loraPending) {
    rf95.send((uint8_t*)msg, (uint8_t)len);
    loraTxBusy = true;
    return;
  }

  memcpy(loraPendingBuf, msg, len);
  loraPendingBuf[len] = '\0';
  loraPendingLen = len;
  loraPending = true;
}

static void serviceLoRaTx() {
  if (!DDS_LORA_TX_ENABLED) return;

  if (loraTxBusy) {
    if (rf95.waitPacketSent(1)) {
      loraTxBusy = false;
    }
  }

  if (!loraTxBusy && loraPending) {
    rf95.send((uint8_t*)loraPendingBuf, (uint8_t)loraPendingLen);
    loraTxBusy = true;
    loraPending = false;
  }
}

// ================== Launch baseline tracking (BOTH sensors) ==================
static bool ax0_valid = false;
static float ax0_hg_g = 0.0f;
static float ax0_lg_g = 0.0f;
static uint32_t baselineStartMs = 0;
static uint32_t launchDwellStartMs = 0;

static void updateAxBaseline(uint32_t now) {
  if (state != FlightState::IDLE) return;
  if (now - baselineStartMs < BASELINE_MIN_MS) return;

  float hgX_g = gFromMps2(safe0(hgAx_mps2));
  float lgX_g = gFromMps2(safe0(lgAx_mps2));

  if (!ax0_valid) {
    ax0_hg_g = hgX_g;
    ax0_lg_g = lgX_g;
    ax0_valid = true;
    return;
  }

  ax0_hg_g = 0.98f * ax0_hg_g + 0.02f * hgX_g;
  ax0_lg_g = 0.98f * ax0_lg_g + 0.02f * lgX_g;
}

static bool launchDetectEitherDeltaX(uint32_t now) {
  if (!ax0_valid) return false;
  if (state != FlightState::IDLE) return false;

  float hgX_g = gFromMps2(safe0(hgAx_mps2));
  float lgX_g = gFromMps2(safe0(lgAx_mps2));

  float dHg = hgX_g - ax0_hg_g;
  float dLg = lgX_g - ax0_lg_g;

  bool trig = (dHg >= LAUNCH_DAX_HG_G_THRESH) || (dLg >= LAUNCH_DAX_LG_G_THRESH);

  if (trig) {
    if (launchDwellStartMs == 0) launchDwellStartMs = now;
    return (now - launchDwellStartMs) >= LAUNCH_DWELL_MS;
  } else {
    launchDwellStartMs = 0;
    return false;
  }
}

// ================== Forward decls ==================
bool initSensors();
bool readSensors();
void updateVz(float alt, uint32_t tnow);
bool positiveTrend(uint32_t dwell_ms);
bool negativeTrend(uint32_t dwell_ms);
void logAndTx_DATA_Compat(const char* reason = nullptr);

// CHANGED: add force parameter
void deployDrogue(const char* reason, bool force=false);
void deployMain(const char* reason, bool force=false);

const char* stateStr(FlightState s);
void handleSerialInput();

// ================== Setup ==================
void setup() {
  pinMode(PIN_DROGUE_1, OUTPUT);
  pinMode(PIN_DROGUE_2, OUTPUT);
  pinMode(PIN_MAIN_1,   OUTPUT);
  pinMode(PIN_MAIN_2,   OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);

  digitalWrite(PIN_DROGUE_1, LOW);
  digitalWrite(PIN_DROGUE_2, LOW);
  digitalWrite(PIN_MAIN_1,   LOW);
  digitalWrite(PIN_MAIN_2,   LOW);
  digitalWrite(PIN_BUZZER,   LOW);

  Serial.begin(115200);
  uint32_t serialStart = millis();
  while (!Serial && (millis() - serialStart < 1500)) delay(10);

  Serial.println(F("\n=== DDS Flight Computer (Backup) ==="));
  Serial.println(F("Output: DATA:<t>:<fields> (nosecone-compatible)"));
  Serial.print(F("LoRa TX Enabled: ")); Serial.println(DDS_LORA_TX_ENABLED ? "YES" : "NO");

  Serial.println(F("Power stabilizing..."));
  delay(POWER_STAB_MS);
  beepFast2_blocking();

  Serial.println(F("Initializing sensors..."));
  sensorsOK = initSensors();
  if (!sensorsOK) {
    Serial.println(F("SENSOR INIT FAILED!"));
    beepLong_blocking();
    state = FlightState::FAULT;
    while (1) { delay(1000); }
  }
  Serial.println(F("Sensors OK"));
  beepFast2_blocking();

  delay(300);
  if (bmp.performReading()) {
    altRef_m = bmp.readAltitude(SEA_LEVEL_HPA);
    Serial.print(F("Altitude reference set: "));
    Serial.print(altRef_m);
    Serial.println(F(" m"));
  } else {
    altRef_m = 0.0f;
    Serial.println(F("Warning: could not set altitude reference"));
  }

  Serial.println(F("Initializing SD card (CS=12)..."));
  if (SD.begin(SD_CS)) {
    char fname[13] = "DDS00.CSV";
    for (int i = 0; i < 100; i++) {
      fname[3] = '0' + (i / 10);
      fname[4] = '0' + (i % 10);
      if (!SD.exists(fname)) { logFile = SD.open(fname, FILE_WRITE); break; }
    }
    if (logFile) {
      logFile.println(
        "Time,"
        "HG_Ax_mps2,HG_Ay_mps2,HG_Az_mps2,"
        "LG_Ax_mps2,LG_Ay_mps2,LG_Az_mps2,"
        "GyroX_dps,GyroY_dps,GyroZ_dps,"
        "Roll_deg,Pitch_deg,Yaw_deg,"
        "BaroAlt_m,Pressure_Pa,Temp_C,"
        "Lat,Lon,GPSAlt_m,Speed_kmph,"
        "MagX_uT,MagY_uT,MagZ_uT,"
        "Battery_V,"
        "State,Event"
      );
      logFile.flush();
      Serial.print(F("Logging to: ")); Serial.println(fname);
    } else {
      Serial.println(F("SD open failed (continuing without SD logging)"));
    }
  } else {
    Serial.println(F("SD init failed (continuing without SD)"));
  }

  t_boot = millis();
  baselineStartMs = t_boot;
  ax0_valid = false;
  launchDwellStartMs = 0;

  // --- NEW: reset launch/AGL/main-arming ---
  t_launch = 0;
  launchAlt_m = 0.0f;
  altAGL_m = 0.0f;
  maxAltAGL_m = 0.0f;
  mainArmed = false;

  state = FlightState::IDLE;
  Serial.println(F("=== READY - State: IDLE ==="));
  Serial.println(F("Launch detect: ΔX (ADXL OR BNO) above baseline + dwell"));
}

// ================== Loop ==================
void loop() {
  const uint32_t now = millis();

  serviceBuzzer(now);
  servicePyro(now);
  serviceLoRaTx();

  if (TEST_MODE && Serial.available()) handleSerialInput();
  if (!TEST_MODE) readSensors();

  if (state == FlightState::IDLE) updateAxBaseline(now);

  const bool drogueTimerElapsed = (t_launch > 0) && (now - t_launch >= DROGUE_BACKUP_MS);
  const bool mainTimerElapsed   = (t_launch > 0) && (now - t_launch >= MAIN_BACKUP_MS);

  switch (state) {
    case FlightState::IDLE: {
      if (launchDetectEitherDeltaX(now) || positiveTrend(ASCENT_POS_VZ_DWELL_MS)) {

        // --- NEW: set launch baseline for AGL and reset max/arming ---
        launchAlt_m = alt_m;
        altAGL_m = 0.0f;
        maxAltAGL_m = 0.0f;
        mainArmed = false;

        t_launch = now;
        state = FlightState::ASCENT_LOCKOUT;

        startRapidBeep(LAUNCH_BEEP_SECONDS);
        logAndTx_DATA_Compat("LAUNCH_DETECTED");
      }
      break;
    }

    case FlightState::ASCENT_LOCKOUT: {
      if (now - t_launch >= FLIGHT_LOCKOUT_MS) {
        state = FlightState::ASCENT;
        Serial.println(F("State -> ASCENT"));
      }
      break;
    }

    case FlightState::ASCENT: {
      if (negativeTrend(APOGEE_NEG_VZ_DWELL_MS)) {
        state = FlightState::APOGEE_DETECT;
        logAndTx_DATA_Compat("APOGEE_NEG_TREND");
      }

      // Timer backup drogue OK (still gated by lockout)
      if (!drogueFired && drogueTimerElapsed && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployDrogue("TIMER_BACKUP_DROGUE", true);
        state = FlightState::DROGUE_DEPLOYED;
      }

      // --- CHANGED: DO NOT allow main timer backup while still in ASCENT ---
      break;
    }

    case FlightState::APOGEE_DETECT: {
      if (!drogueFired && (vz_mps <= 0.0f) && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployDrogue("APOGEE", false);
        state = FlightState::DROGUE_DEPLOYED;
      }

      if (!drogueFired && drogueTimerElapsed && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployDrogue("TIMER_BACKUP_DROGUE", true);
        state = FlightState::DROGUE_DEPLOYED;
      }
      break;
    }

    case FlightState::DROGUE_DEPLOYED: {
      static uint32_t downFastStart = 0;

      // If falling too fast, drogue might have failed -> force main
      if (vz_mps <= MAX_FALL_SPEED_FOR_DROGUE) {
        if (downFastStart == 0) downFastStart = now;
        if (!mainFired && (now - downFastStart >= DROGUE_INEFFECTIVE_MS)
            && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
          deployMain("DROGUE_INEFFECTIVE_FALLRATE", true);
          state = FlightState::MAIN_DEPLOYED;
          break;
        }
      } else {
        downFastStart = 0;
      }

      // --- CHANGED: MAIN based on AGL AND must be "armed" (must have gone above main once) ---
      if (!mainFired
          && mainArmed
          && (altAGL_m <= MAIN_ALT_M)
          && (vz_mps <= 0.0f)
          && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployMain("MAIN_AglCross", true);
        state = FlightState::MAIN_DEPLOYED;
        break;
      }

      // --- NEW: MAIN timer backup only AFTER drogue has fired ---
      if (!mainFired
          && drogueFired
          && mainTimerElapsed
          && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployMain("TIMER_BACKUP_MAIN_AFTER_DROGUE", true);
        state = FlightState::MAIN_DEPLOYED;
        break;
      }

      break;
    }

    case FlightState::MAIN_DEPLOYED:
    case FlightState::LANDED:
    case FlightState::FAULT:
    default:
      break;
  }

  static uint32_t t_lastTX = 0;
  if (now - t_lastTX > 100) {
    logAndTx_DATA_Compat(nullptr);
    t_lastTX = now;
  }
}

// ================== Sensor Init ==================
bool initSensors() {
  bool ok = true;

  // ----- RFM95 -----
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println(F("RFM95 init failed"));
    ok = false;
  } else {
    Serial.println(F("RFM95 init OK"));

    if (!rf95.setFrequency(RF95_FREQ_MHZ)) {
      Serial.println(F("RFM95 freq set failed"));
      ok = false;
    } else {
      Serial.print(F("RFM95 freq OK: "));
      Serial.println(RF95_FREQ_MHZ, 3);
    }

    // MUST match receiver:
    // applyLoRaParams(): BW 500k, CR 4/5, SF7, CRC ON
    rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128);
    rf95.setPayloadCRC(true);
    rf95.setPreambleLength(8);

    rf95.setTxPower(RF95_TX_POWER_DBM, false);
  }

  // ----- BMP390 -----
  if (!bmp.begin_I2C(BMP3XX_I2C_ADDR)) {
    Serial.println(F("BMP390 init failed"));
    ok = false;
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // ----- BNO08x -----
  if (!bno08x.begin_I2C()) {
    Serial.println(F("BNO08x init failed"));
    ok = false;
  } else {
    bool repOK = true;
    repOK &= bno08x.enableReport(SH2_ROTATION_VECTOR, 5000);
    repOK &= bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, 5000);
    repOK &= bno08x.enableReport(SH2_ACCELEROMETER, 5000);
    repOK &= bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, 5000);
    if (!repOK) {
      Serial.println(F("BNO08x enable report failed"));
      ok = false;
    }
  }

  // ----- ADXL375 -----
  if (!adxl.begin()) {
    Serial.println(F("ADXL375 init failed"));
    ok = false;
  } else {
    adxl.setDataRate(ADXL3XX_DATARATE_100_HZ);
  }

  return ok;
}

// ================== Sensor Read ==================
bool readSensors() {
  const uint32_t now = millis();

  if (bmp.performReading()) {
    float absAlt = bmp.readAltitude(SEA_LEVEL_HPA);
    alt_m = absAlt - altRef_m;   // still "relative to boot reference"
  }

  // --- NEW: compute AGL only after launch baseline captured ---
  if (t_launch > 0) {
    altAGL_m = alt_m - launchAlt_m;

    if (altAGL_m > maxAltAGL_m) maxAltAGL_m = altAGL_m;

    // arm main only after you've CLEARLY exceeded the threshold
    if (!mainArmed && maxAltAGL_m > (MAIN_ALT_M + MAIN_ARM_MARGIN_M)) {
      mainArmed = true;
    }
  } else {
    altAGL_m = 0.0f;
    maxAltAGL_m = 0.0f;
    mainArmed = false;
  }

  sensors_event_t adxlEvt;
  adxl.getEvent(&adxlEvt);
  hgAx_mps2 = safe0(adxlEvt.acceleration.x);
  hgAy_mps2 = safe0(adxlEvt.acceleration.y);
  hgAz_mps2 = safe0(adxlEvt.acceleration.z);

  float ax_g = gFromMps2(hgAx_mps2);
  float ay_g = gFromMps2(hgAy_mps2);
  float az_g = gFromMps2(hgAz_mps2);
  am_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

  uint32_t start = millis();
  bool gotRot=false, gotAcc=false, gotGyro=false, gotMag=false;

  while ((millis() - start < 20) && !(gotRot && gotAcc && gotGyro && gotMag)) {
    if (bno08x.getSensorEvent(&bnoValue)) {
      switch (bnoValue.sensorId) {
        case SH2_ROTATION_VECTOR: {
          float qi = bnoValue.un.rotationVector.i;
          float qj = bnoValue.un.rotationVector.j;
          float qk = bnoValue.un.rotationVector.k;
          float qr = bnoValue.un.rotationVector.real;

          roll_deg  = atan2f(2.0f * (qr * qi + qj * qk),
                             1.0f - 2.0f * (qi * qi + qj * qj)) * 180.0f / PI;
          pitch_deg = asinf(2.0f * (qr * qj - qk * qi)) * 180.0f / PI;
          yaw_deg   = atan2f(2.0f * (qr * qk + qi * qj),
                             1.0f - 2.0f * (qj * qj + qk * qk)) * 180.0f / PI;
          yaw_deg = wrap360f(yaw_deg);
          gotRot = true;
        } break;

        case SH2_ACCELEROMETER:
          lgAx_mps2 = safe0(bnoValue.un.accelerometer.x);
          lgAy_mps2 = safe0(bnoValue.un.accelerometer.y);
          lgAz_mps2 = safe0(bnoValue.un.accelerometer.z);
          gotAcc = true;
          break;

        case SH2_GYROSCOPE_CALIBRATED:
          gyroX_dps = safe0(bnoValue.un.gyroscope.x * 180.0f / PI);
          gyroY_dps = safe0(bnoValue.un.gyroscope.y * 180.0f / PI);
          gyroZ_dps = safe0(bnoValue.un.gyroscope.z * 180.0f / PI);
          gotGyro = true;
          break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
          magX_uT = safe0(bnoValue.un.magneticField.x);
          magY_uT = safe0(bnoValue.un.magneticField.y);
          magZ_uT = safe0(bnoValue.un.magneticField.z);
          gotMag = true;
          break;
      }
    }
    yield();
  }

  const int VBATPIN = A7;
  analogReference(AR_DEFAULT);
  uint16_t raw = analogRead(VBATPIN);
  float v = raw;
  v *= 2.0f;
  v *= 3.3f;
  v /= 1024.0f;
  batt_V = safe0(v);

  // NOTE: you can choose whether Vz is based on alt_m or altAGL_m.
  // For flight logic it usually doesn’t matter (difference is constant),
  // but if you want consistency post-launch, use altAGL_m after launch:
  float vzAlt = (t_launch > 0) ? altAGL_m : alt_m;
  updateVz(vzAlt, now);

  return true;
}

// ================== Vz calc ==================
void updateVz(float alt, uint32_t tnow) {
  ring[ringHead] = {tnow, alt};
  ringHead = (ringHead + 1) % VZ_WIN;
  if (ringCount < VZ_WIN) ringCount++;

  if (ringCount >= 2) {
    const int oldest = (ringHead + VZ_WIN - ringCount) % VZ_WIN;
    const Sample &a = ring[oldest];
    const Sample &b = ring[(ringHead + VZ_WIN - 1) % VZ_WIN];
    float dt = (b.t - a.t) / 1000.0f;
    if (dt > 0.0f) vz_mps = (b.alt - a.alt) / dt;
  }
}

bool positiveTrend(uint32_t dwell_ms) {
  static uint32_t posStart = 0;
  if (vz_mps > ASCENT_POS_VZ_THRESH) {
    if (posStart == 0) posStart = millis();
    return (millis() - posStart) >= dwell_ms;
  } else { posStart = 0; return false; }
}

bool negativeTrend(uint32_t dwell_ms) {
  static uint32_t negStart = 0;
  if (vz_mps < APOGEE_NEG_VZ_THRESH) {
    if (negStart == 0) negStart = millis();
    return (millis() - negStart) >= dwell_ms;
  } else { negStart = 0; return false; }
}

// ================== Deploy ==================
void deployDrogue(const char* reason, bool force) {
  if (drogueFired) return;
  if (!force && (vz_mps > 0.0f)) return;
  if (pyroActive()) return;

  drogueFired = true;
  t_drogue_fire = millis();

  startRapidBeep(1);
  Serial.print(F(">>> PYRO DROGUE CMD: ")); Serial.println(reason ? reason : "-");

  logAndTx_DATA_Compat(reason ? reason : "DROGUE_DEPLOY");
  startPyroSequence(PYRO_DROGUE, PIN_DROGUE_1, PIN_DROGUE_2);
}

void deployMain(const char* reason, bool force) {
  if (mainFired) return;
  if (!force && (vz_mps > 0.0f)) return;
  if (pyroActive()) return;

  mainFired = true;
  t_main_fire = millis();

  startRapidBeep(1);
  Serial.print(F(">>> PYRO MAIN CMD: ")); Serial.println(reason ? reason : "-");

  logAndTx_DATA_Compat(reason ? reason : "MAIN_DEPLOY");
  startPyroSequence(PYRO_MAIN, PIN_MAIN_1, PIN_MAIN_2);
}

const char* stateStr(FlightState s) {
  switch (s) {
    case FlightState::BOOT: return "BOOT";
    case FlightState::IDLE: return "IDLE";
    case FlightState::ASCENT_LOCKOUT: return "ASCENT_LOCKOUT";
    case FlightState::ASCENT: return "ASCENT";
    case FlightState::APOGEE_DETECT: return "APOGEE";
    case FlightState::DROGUE_DEPLOYED: return "DROGUE";
    case FlightState::MAIN_DEPLOYED: return "MAIN";
    case FlightState::LANDED: return "LANDED";
    case FlightState::FAULT: return "FAULT";
  }
  return "UNK";
}

// ================== DATA formatting ==================
void logAndTx_DATA_Compat(const char* reason) {
  const uint32_t now = millis();

  float tempC = 0.0f, presPa = 0.0f;
  if (bmp.performReading()) {
    tempC  = safe0(bmp.temperature);
    presPa = safe0(bmp.pressure);
  }

  // --- CHANGED: transmit AGL altitude once launch happens ---
  float outAlt_m = (t_launch > 0) ? altAGL_m : alt_m;

  char dataPkt[360];
  snprintf(dataPkt, sizeof(dataPkt),
    "DATA:%lu:"
    "%.1f:%.1f:%.1f:"
    "%.1f:%.1f:%.1f:"
    "%.1f:%.1f:%.1f:"
    "%.1f:%.1f:%.1f:"
    "%.1f:%.0f:%.1f:"
    "%.6f:%.6f:%.1f:%.1f:"
    "%.1f:%.1f:%.1f:"
    "%.2f",
    (unsigned long)now,
    safe0(hgAx_mps2), safe0(hgAy_mps2), safe0(hgAz_mps2),
    safe0(lgAx_mps2), safe0(lgAy_mps2), safe0(lgAz_mps2),
    safe0(gyroX_dps), safe0(gyroY_dps), safe0(gyroZ_dps),
    safe0(roll_deg),  safe0(pitch_deg), safe0(yaw_deg),
    safe0(outAlt_m),  safe0(presPa),    safe0(tempC),
    (double)lat, (double)lon, safe0(gpsAlt_m), safe0(gpsSpeed_kmph),
    safe0(magX_uT), safe0(magY_uT), safe0(magZ_uT),
    safe0(batt_V)
  );

  if (Serial) Serial.println(dataPkt);

  if (logFile) {
    char csv[460];
    snprintf(csv, sizeof(csv),
      "%lu,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,%.0f,%.2f,"
      "%.6f,%.6f,%.2f,%.2f,"
      "%.2f,%.2f,%.2f,"
      "%.2f,"
      "%s,%s\n",
      (unsigned long)now,
      safe0(hgAx_mps2), safe0(hgAy_mps2), safe0(hgAz_mps2),
      safe0(lgAx_mps2), safe0(lgAy_mps2), safe0(lgAz_mps2),
      safe0(gyroX_dps), safe0(gyroY_dps), safe0(gyroZ_dps),
      safe0(roll_deg), safe0(pitch_deg), safe0(yaw_deg),
      safe0(outAlt_m), safe0(presPa), safe0(tempC),
      (double)lat, (double)lon, safe0(gpsAlt_m), safe0(gpsSpeed_kmph),
      safe0(magX_uT), safe0(magY_uT), safe0(magZ_uT),
      safe0(batt_V),
      stateStr(state),
      (reason ? reason : "-")
    );
    logFile.print(csv);
    static uint32_t lastFlush = 0;
    if (now - lastFlush > 250) { logFile.flush(); lastFlush = now; }
  }

  queueLoRaTx(dataPkt);
}

void handleSerialInput() {
  // optional; keep for later
}
