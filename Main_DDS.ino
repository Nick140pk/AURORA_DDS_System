#include <Arduino.h>
#include <SPI.h>
#include <RH_RF95.h>
#include <Adafruit_BNO08x.h>
#include <math.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_ADXL375.h>
#include <SD.h>

// ================== Pins (from your spec) ==================
constexpr uint8_t PIN_DROGUE_1 = 5;
constexpr uint8_t PIN_DROGUE_2 = 6;
constexpr uint8_t PIN_MAIN_1   = 9;
constexpr uint8_t PIN_MAIN_2   = 10;
constexpr uint8_t PIN_BUZZER   = 11;

// ================== TEST MODE ==================
// Set to true to enable serial test mode (inject data via serial)
// Set to false for normal flight operation
constexpr bool TEST_MODE = false;

// ================== Parameters ==================
constexpr float     MAIN_ALT_M               = 200.0f;   // m AGL
constexpr uint32_t  DROGUE_BACKUP_MS        = 15000;    // 15 s
constexpr uint32_t  MAIN_BACKUP_MS          = 25000;    // 25 s
constexpr float     G_LOCKOUT               = 2.0f;     // g
constexpr uint32_t  POWER_STAB_MS           = 2000;
constexpr uint32_t  FLIGHT_LOCKOUT_MS       = 10000;    // post-launch no-deploy window
constexpr float     APOGEE_NEG_VZ_THRESH    = -1.5f;    // m/s
constexpr uint32_t  APOGEE_NEG_VZ_DWELL_MS  = 250;      // ms
constexpr float     ASCENT_POS_VZ_THRESH    = +2.0f;    // m/s
constexpr uint32_t  ASCENT_POS_VZ_DWELL_MS  = 250;      // ms
constexpr float     MAX_FALL_SPEED_FOR_DROGUE = -60.0f; // m/s (negative = down)
constexpr uint32_t  DROGUE_INEFFECTIVE_MS   = 400;      // ms
constexpr uint16_t  PYRO_PULSE_MS           = 2000;     // ms - 4 seconds for reliable ignition

// ================== State machine ==================
enum class FlightState {
  BOOT, IDLE, ASCENT_LOCKOUT, ASCENT, APOGEE_DETECT,
  DROGUE_DEPLOYED, MAIN_DEPLOYED, LANDED, FAULT
};
FlightState state = FlightState::BOOT;

// ================== Globals ==================
uint32_t t_boot, t_launch = 0, t_lastSample = 0;
uint32_t t_drogue_fire = 0, t_main_fire = 0;
bool drogueFired = false, mainFired = false;
bool sensorsOK = false;

float alt_m = 0, altRef_m = 0, vz_mps = 0;   // baro-derived
float ax_g = 0, ay_g = 0, az_g = 0, am_g = 0; // hi-g accel magnitude
float batt_V = 0;
bool haveGPS = false; // set true if you attach GPS; include lat/lon/alt

// Serial state
bool serialAvailable = false;

// Simple moving window for vz estimate
const int VZ_WIN = 8; // ~8 samples ≈160ms if ~50Hz
struct Sample { uint32_t t; float alt; };
Sample ring[VZ_WIN]; int ringHead = 0; int ringCount = 0;

// ================== Peripherals & wiring ==================
// ---- LoRa RadioHead RH_RF95 on Feather M0 LoRa default pins ----
constexpr uint8_t RFM95_CS  = 8;
constexpr uint8_t RFM95_IRQ = 3;
constexpr uint8_t RFM95_RST = 4;
constexpr float   RF95_FREQ_MHZ = 915.0;

RH_RF95 rf95(RFM95_CS, RFM95_IRQ);

// ---- SD card (Adalogger FeatherWing CS moved to D12) ----
constexpr uint8_t SD_CS = 12;  // Moved to D12 to avoid conflict with PIN_MAIN_2
File logFile;

// ---- Sensors ----
Adafruit_BMP3XX bmp;            // I2C by default
Adafruit_BNO08x bno08x;         // I2C by default
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345); // unique ID optional

// BMP390 settings
#define BMP3XX_I2C_ADDR 0x77
constexpr float SEA_LEVEL_HPA = 1013.25f;  // Adjust to field QNH if known

// ================== Forward declarations ==================
void beepFast2();
void beepLong();
void beepShort();
void preFireBeep(); // NEW
void pyroPulse(uint8_t pin);
bool initSensors();
bool readSensors(); // fills alt_m, ax/ay/az, am_g, batt_V, vz_mps
void updateVz(float alt, uint32_t tnow);
bool positiveTrend(uint32_t dwell_ms);
bool negativeTrend(uint32_t dwell_ms);
void logAndTx(const char* reason = nullptr);
void deployDrogue(const char* reason);
void deployMain(const char* reason);
const char* stateStr(FlightState s);
void serialPrint(const char* msg);
void serialPrintln(const char* msg);
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

  // Initialize serial without blocking
  Serial.begin(115200);
  // Wait up to 2000ms for serial connection
  uint32_t serialStart = millis();
  while (!Serial && (millis() - serialStart < 2000)) {
    delay(10);
  }
  // Always assume serial is available for output
  serialAvailable = true;
  
  Serial.println(F("\n=== Rocket Flight Computer ==="));
  Serial.print(F("Test Mode: "));
  Serial.println(TEST_MODE ? "ENABLED" : "DISABLED");
  if (TEST_MODE) {
    Serial.println(F("Commands:"));
    Serial.println(F("  ALT:xxx.xx - Set altitude (m AGL)"));
    Serial.println(F("  VZ:xx.xx - Set vertical velocity (m/s)"));
    Serial.println(F("  ACC:x.xx - Set accel magnitude (g)"));
    Serial.println(F("  LAUNCH - Trigger launch detection"));
    Serial.println(F("  STATUS - Print current state"));
  }

  // Power stabilization
  serialPrintln("Power stabilizing...");
  delay(POWER_STAB_MS);
  beepFast2();

  // Initialize peripherals
  serialPrintln("Initializing sensors...");
  sensorsOK = initSensors();
  if (!sensorsOK) {
    serialPrintln("SENSOR INIT FAILED!");
    beepLong();  // fatal
    state = FlightState::FAULT;
    while (1) { delay(1000); }
  } else {
    serialPrintln("Sensors OK");
    beepFast2();
  }

  // Zero altitude reference after a short settle
  delay(500);
  // Set relative altitude reference to current reading:
  if (bmp.performReading()) {
    float alt0 = bmp.readAltitude(SEA_LEVEL_HPA);
    altRef_m = alt0;  // Later alt_m = currentAlt - altRef_m => AGL-ish
    if (serialAvailable) {
      Serial.print(F("Altitude reference set: "));
      Serial.print(altRef_m);
      Serial.println(F(" m"));
    }
  } else {
    altRef_m = 0.0f;
    serialPrintln("Warning: Could not set altitude reference");
  }

  // Prepare log file
  serialPrintln("Initializing SD card...");
  if (SD.begin(SD_CS)) {
    // Unique filename: LOG00.CSV..LOG99.CSV
    char fname[13] = "LOG00.CSV";
    for (int i = 0; i < 100; i++) {
      fname[3] = '0' + (i / 10);
      fname[4] = '0' + (i % 10);
      if (!SD.exists(fname)) {
        logFile = SD.open(fname, FILE_WRITE);
        break;
      }
    }
    if (logFile) {
      logFile.println(F("time_ms,state,alt_m,vz_mps,ax_g,ay_g,az_g,am_g,batt_V,lat,lon,gpsAlt,evt,reason"));
      logFile.flush();
      if (serialAvailable) {
        Serial.print(F("Logging to: "));
        Serial.println(fname);
      }
    }
  } else {
    serialPrintln("SD card init failed (continuing without SD)");
  }

  t_boot = millis();
  state = FlightState::IDLE;
  serialPrintln("=== READY - State: IDLE ===\n");
}

// ================== Loop ==================
void loop() {
  const uint32_t now = millis();

  // Handle serial input for test mode
  if (TEST_MODE && serialAvailable && Serial.available()) {
    handleSerialInput();
  }

  // Read sensors (or use injected test data)
  if (!TEST_MODE) {
    if (!readSensors()) {
      // non-fatal: mark degraded sensor flags here if needed
    }
  }

  // Basic timers
  const bool launchTimerElapsed = (t_launch > 0) && (now - t_launch >= MAIN_BACKUP_MS);
  const bool drogueTimerElapsed = (t_launch > 0) && (now - t_launch >= DROGUE_BACKUP_MS);

  switch (state) {
    case FlightState::IDLE: {
      // Looking for launch: positive trend OR accel > G_LOCKOUT briefly
      if (positiveTrend(ASCENT_POS_VZ_DWELL_MS) || (am_g > G_LOCKOUT)) {
        t_launch = now;
        state = FlightState::ASCENT_LOCKOUT;
        beepLong(); // LAUNCH DETECTED BEEP
        logAndTx("LAUNCH_DETECTED");
      }
      break;
    }

    case FlightState::ASCENT_LOCKOUT: {
      // Ignore any deploys for FLIGHT_LOCKOUT_MS
      if (now - t_launch >= FLIGHT_LOCKOUT_MS) {
        state = FlightState::ASCENT;
        serialPrintln("State -> ASCENT");
      }
      break;
    }

    case FlightState::ASCENT: {
      // Watch for apogee
      if (negativeTrend(APOGEE_NEG_VZ_DWELL_MS)) {
        state = FlightState::APOGEE_DETECT;
        logAndTx("APOGEE_NEG_TREND");
      }
      // Backup timers - drogue first, then main if needed
      if (!drogueFired && drogueTimerElapsed && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployDrogue("TIMER_BACKUP_DROGUE");
        state = FlightState::DROGUE_DEPLOYED;
      }
      else if (!mainFired && launchTimerElapsed && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployMain("TIMER_BACKUP_MAIN");
        state = FlightState::MAIN_DEPLOYED;
      }
      break;
    }

    case FlightState::APOGEE_DETECT: {
      // Only deploy drogue when descending (vz <= 0) and outside lockout
      if (!drogueFired && (vz_mps <= 0.0f) && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployDrogue("APOGEE");
        state = FlightState::DROGUE_DEPLOYED;
      }
      // Backup if apogee logic lingers
      if (!drogueFired && drogueTimerElapsed && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
        deployDrogue("TIMER_BACKUP_DROGUE");
        state = FlightState::DROGUE_DEPLOYED;
      }
      break;
    }

    case FlightState::DROGUE_DEPLOYED: {
      // If drogue ineffective (still > |60| m/s down for a little while), cut main
      static uint32_t downFastStart = 0;
      if (vz_mps <= MAX_FALL_SPEED_FOR_DROGUE) {
        if (downFastStart == 0) downFastStart = now;
        if (!mainFired && (now - downFastStart >= DROGUE_INEFFECTIVE_MS)
            && (now - t_launch >= FLIGHT_LOCKOUT_MS)) {
          deployMain("DROGUE_INEFFECTIVE_FALLRATE");
          state = FlightState::MAIN_DEPLOYED;
        }
      } else {
        downFastStart = 0;
      }

      // Nominal altitude-based main
      if (!mainFired && (alt_m <= MAIN_ALT_M) && (vz_mps <= 0.0f)
          && (now - t_launch >= MAIN_BACKUP_MS)) {
        deployMain("MAIN_ALTITUDE");
        state = FlightState::MAIN_DEPLOYED;
      }

      break;
    }

    case FlightState::MAIN_DEPLOYED: {
      // Detect landing: |vz| small for a few seconds and accel ~1g
      static uint32_t stillStart = 0;
      const bool still = (fabs(vz_mps) < 0.2f) && (am_g > 0.7f && am_g < 1.3f);
      if (still) {
        if (stillStart == 0) stillStart = now;
        if (now - stillStart > 3000) {
          state = FlightState::LANDED;
          logAndTx("LANDED");
        }
      } else stillStart = 0;
      break;
    }

    case FlightState::LANDED:
    case FlightState::FAULT:
    default:
      // Keep logging/tx heartbeat if desired
      break;
  }

  // Always log/tx heartbeat at some rate
  static uint32_t t_lastTX = 0;
  if (now - t_lastTX > 100) { // 10 Hz
    logAndTx(nullptr);
    t_lastTX = now;
  }
}

// ================== Helpers ==================
void serialPrint(const char* msg) {
  Serial.print(msg);
}

void serialPrintln(const char* msg) {
  Serial.println(msg);
}

void beepFast2() {
  for (int i=0;i<2;i++){ digitalWrite(PIN_BUZZER, HIGH); delay(80); digitalWrite(PIN_BUZZER, LOW); delay(80); }
}
void beepLong() { 
  digitalWrite(PIN_BUZZER, HIGH); 
  delay(1200); 
  digitalWrite(PIN_BUZZER, LOW);
}

void beepShort() {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(200);
  digitalWrite(PIN_BUZZER, LOW);
}

// ===== NEW: one long warning beep right before firing any pyro =====
void preFireBeep() {
  if (serialAvailable) Serial.println(F("*** WARNING: PYRO FIRE IMMINENT ***"));
  beepLong();          // ~1.2 s ON
  delay(150);          // brief pause to end tone cleanly
}

void pyroPulse(uint8_t pin) {
  // NOTE: ensure you have proper pyro driver/transistor + current source
  digitalWrite(pin, HIGH);
  delay(PYRO_PULSE_MS);
  digitalWrite(pin, LOW);
}

bool initSensors() {
  bool ok = true;

  // ----- SD (non-fatal): setup() already did SD.begin; skip or retry if you want -----
  // if (!SD.begin(SD_CS)) { serialPrintln("SD init failed (non-fatal)"); }

  // ----- RFM95 -----
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  if (!rf95.init()) {
    serialPrintln("RFM95 init failed");
    ok = false;
  }
  if (ok) {
    if (!rf95.setFrequency(RF95_FREQ_MHZ)) {
      serialPrintln("RFM95 freq set failed");
      ok = false;
    }
    rf95.setTxPower(13, false);
  }

  // ----- BMP390 -----
  if (!bmp.begin_I2C(BMP3XX_I2C_ADDR)) {
    serialPrintln("BMP390 init failed");
    ok = false;
  }
  if (ok) {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
  }

  // ----- BNO085 (non-fatal) -----
  if (!bno08x.begin_I2C()) {
    serialPrintln("BNO085 init failed (non-fatal)");
    // Don't set ok = false - BNO is optional
  }

  // ----- ADXL375 (High-g accel) -----
  if (!adxl.begin()) {
    serialPrintln("ADXL375 init failed");
    ok = false;
  } else {
    // ADXL375 is fixed at 200g range
    adxl.setDataRate(ADXL3XX_DATARATE_100_HZ);
  }

  return ok;
}

bool readSensors() {
  const uint32_t now = millis();

  // ----- BMP390 altitude -----
  if (bmp.performReading()) {
    float absAlt = bmp.readAltitude(SEA_LEVEL_HPA);
    alt_m = absAlt - altRef_m;
  }

  // ----- ADXL375 accel -> g -----
  sensors_event_t event;
  adxl.getEvent(&event);
  ax_g = event.acceleration.x / 9.80665f;
  ay_g = event.acceleration.y / 9.80665f;
  az_g = event.acceleration.z / 9.80665f;
  am_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);

  // ----- Battery voltage -----
  const int VBATPIN = A7;
  analogReference(AR_DEFAULT);
  uint16_t raw = analogRead(VBATPIN);
  float v = raw;
  v *= 2.0f;
  v *= 3.3f;
  v /= 1024.0f;
  batt_V = v;

  // ----- Vertical speed estimate -----
  updateVz(alt_m, now);

  return true;
}

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
  if (vz_mps > ASCENT_POS_VZ_THRESH || am_g > G_LOCKOUT) {
    if (posStart == 0) posStart = millis();
    return (millis() - posStart) >= dwell_ms;
  } else {
    posStart = 0;
    return false;
  }
}

bool negativeTrend(uint32_t dwell_ms) {
  static uint32_t negStart = 0;
  if (vz_mps < APOGEE_NEG_VZ_THRESH) {
    if (negStart == 0) negStart = millis();
    return (millis() - negStart) >= dwell_ms;
  } else {
    negStart = 0;
    return false;
  }
}

void deployDrogue(const char* reason) {
  if (drogueFired) return;
  if (vz_mps > 0.0f) return;

  preFireBeep();                // <<< NEW WARNING BEEP

  pyroPulse(PIN_DROGUE_1);
  pyroPulse(PIN_DROGUE_2);
  drogueFired = true;
  t_drogue_fire = millis();
  logAndTx(reason ? reason : "DROGUE_DEPLOY");
  
  if (serialAvailable) {
    Serial.print(F(">>> DROGUE DEPLOYED: "));
    Serial.println(reason ? reason : "DROGUE_DEPLOY");
  }
}

void deployMain(const char* reason) {
  if (mainFired) return;
  if (vz_mps > 0.0f) return;

  preFireBeep();               // <<< NEW WARNING BEEP

  pyroPulse(PIN_MAIN_1);
  pyroPulse(PIN_MAIN_2);
  mainFired = true;
  t_main_fire = millis();
  logAndTx(reason ? reason : "MAIN_DEPLOY");
  
  if (serialAvailable) {
    Serial.print(F(">>> MAIN DEPLOYED: "));
    Serial.println(reason ? reason : "MAIN_DEPLOY");
  }
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

void logAndTx(const char* reason) {
  const uint32_t now = millis();
  const char* st = stateStr(state);

  float lat=0, lon=0, gpsAlt=0;

  char line[192];
  snprintf(line, sizeof(line),
    "%lu,%s,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.6f,%.6f,%.1f,%s,%s\n",
    (unsigned long)now, st, alt_m, vz_mps, ax_g, ay_g, az_g, am_g, batt_V,
    lat, lon, gpsAlt,
    drogueFired?"DROGUE":"-", reason?reason:"-");

  // ---- SD write ----
  if (logFile) {
    logFile.print(line);
    static uint32_t lastFlush = 0;
    if (now - lastFlush > 200) {
      logFile.flush();
      lastFlush = now;
    }
  }

  // ---- Serial output ----
  if (serialAvailable && Serial) {
    Serial.print(line);
  }

  // ---- LoRa send ----
  uint8_t len = min((int)strlen(line), 240);
  rf95.send((uint8_t*)line, len);
  rf95.waitPacketSent();
}

void handleSerialInput() {
  static char buffer[64];
  static int bufIdx = 0;
  
  while (Serial.available()) {
    char c = Serial.read();
    
    if (c == '\n' || c == '\r') {
      if (bufIdx > 0) {
        buffer[bufIdx] = '\0';
        
        // Parse commands
        if (strncmp(buffer, "ALT:", 4) == 0) {
          float val = atof(buffer + 4);
          alt_m = val;
          updateVz(alt_m, millis());
          Serial.print(F("Set altitude: "));
          Serial.print(alt_m);
          Serial.println(F(" m"));
        }
        else if (strncmp(buffer, "VZ:", 3) == 0) {
          vz_mps = atof(buffer + 3);
          Serial.print(F("Set vz: "));
          Serial.print(vz_mps);
          Serial.println(F(" m/s"));
        }
        else if (strncmp(buffer, "ACC:", 4) == 0) {
          am_g = atof(buffer + 4);
          Serial.print(F("Set accel: "));
          Serial.print(am_g);
          Serial.println(F(" g"));
        }
        else if (strcmp(buffer, "LAUNCH") == 0) {
          Serial.println(F("*** FORCING LAUNCH ***"));
          t_launch = millis();
          state = FlightState::ASCENT_LOCKOUT;
          logAndTx("TEST_LAUNCH");
        }
        else if (strcmp(buffer, "BEEP") == 0) {
          Serial.println(F("*** TESTING BEEP ***"));
          beepLong();
          Serial.println(F("*** BEEP DONE ***"));
        }
        else if (strcmp(buffer, "BEEP2") == 0) {
          Serial.println(F("*** TESTING FAST BEEPS ***"));
          beepFast2();
          Serial.println(F("*** FAST BEEPS DONE ***"));
        }
        else if (strcmp(buffer, "STATUS") == 0) {
          Serial.println(F("\n=== STATUS ==="));
          Serial.print(F("State: ")); Serial.println(stateStr(state));
          Serial.print(F("Alt: ")); Serial.print(alt_m); Serial.println(F(" m"));
          Serial.print(F("Vz: ")); Serial.print(vz_mps); Serial.println(F(" m/s"));
          Serial.print(F("Accel: ")); Serial.print(am_g); Serial.println(F(" g"));
          Serial.print(F("Battery: ")); Serial.print(batt_V); Serial.println(F(" V"));
          Serial.print(F("Drogue: ")); Serial.println(drogueFired ? "FIRED" : "ARMED");
          Serial.print(F("Main: ")); Serial.println(mainFired ? "FIRED" : "ARMED");
          if (t_launch > 0) {
            Serial.print(F("T+ ")); 
            Serial.print((millis() - t_launch) / 1000.0, 1); 
            Serial.println(F(" s"));
          }
          Serial.println(F("==============\n"));
        }
        else {
          Serial.print(F("Unknown command: "));
          Serial.println(buffer);
        }
        
        bufIdx = 0;
      }
    }
    else if (bufIdx < 63) {
      buffer[bufIdx++] = c;
    }
  }
}
