// === Simple Sensor Printer (Feather M0 + BMP390 + BNO08x + ADXL375) ===
// Opens Serial at 115200 and prints CSV lines ~10 Hz.
//
// Wiring: All sensors on I2C (SDA/SCL). Battery sense on A7 (Feather M0).
// Libraries (Adafruit versions):
//  - Adafruit BMP3XX Library
//  - Adafruit BNO08x
//  - Adafruit Unified Sensor
//  - Adafruit ADXL375
//
// If your BMP390 uses 0x76 instead of 0x77, change BMP3XX_I2C_ADDR.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL375.h>
#include <math.h>

// ---------- Barometer (BMP390/3XX) ----------
#define BMP3XX_I2C_ADDR 0x77
constexpr float SEA_LEVEL_HPA = 1013.25f;  // set to field QNH if known
Adafruit_BMP3XX bmp;

// ---------- IMU (BNO08x) ----------
Adafruit_BNO08x bno08x;
sh2_SensorValue_t bnoVal;
bool haveBNO = false;

// We'll compute Euler from rotation vector (quaternion)
float yaw_deg = NAN, pitch_deg = NAN, roll_deg = NAN;

// ---------- High-g accelerometer (ADXL375) ----------
Adafruit_ADXL375 adxl = Adafruit_ADXL375(12345);
float ax_g = NAN, ay_g = NAN, az_g = NAN, am_g = NAN;

// ---------- Altitude reference ----------
float altRef_m = NAN;

// ---------- Battery sense ----------
constexpr int VBATPIN = A7;
float batt_V = NAN;

// ---------- Print rate ----------
constexpr uint32_t PRINT_PERIOD_MS = 100; // ~10 Hz
uint32_t t_lastPrint = 0;

// ---- Helpers ----
static void enableBNOReports() {
  // Rotation vector for orientation; accel & gyro are optional but handy
  bno08x.enableReport(SH2_ROTATION_VECTOR);
  bno08x.enableReport(SH2_ACCELEROMETER);
  bno08x.enableReport(SH2_RAW_GYROSCOPE);
}

// Convert quaternion to yaw/pitch/roll (degrees)
// Based on typical aerospace ZYX convention: yaw (Z), pitch (Y), roll (X)
static void quatToYPR(float qr, float qi, float qj, float qk,
                      float &yaw, float &pitch, float &roll) {
  // Normalize just in case
  float n = sqrtf(qr*qr + qi*qi + qj*qj + qk*qk);
  if (n <= 0) { yaw = pitch = roll = NAN; return; }
  qr /= n; qi /= n; qj /= n; qk /= n;

  // ZYX Euler
  float siny_cosp = 2.0f * (qr*qk + qi*qj);
  float cosy_cosp = 1.0f - 2.0f * (qj*qj + qk*qk);
  float yaw_rad   = atan2f(siny_cosp, cosy_cosp);

  float sinp = 2.0f * (qr*qj - qk*qi);
  float pitch_rad = (fabsf(sinp) >= 1.0f) ? copysignf(M_PI/2.0f, sinp) : asinf(sinp);

  float sinr_cosp = 2.0f * (qr*qi + qj*qk);
  float cosr_cosp = 1.0f - 2.0f * (qi*qi + qj*qj);
  float roll_rad  = atan2f(sinr_cosp, cosr_cosp);

  yaw   = yaw_rad   * 180.0f / PI;
  pitch = pitch_rad * 180.0f / PI;
  roll  = roll_rad  * 180.0f / PI;
}

void setup() {
  // Serial
  Serial.begin(115200);
  // Give USB up to 2s to enumerate (won't block forever on battery)
  uint32_t t0 = millis();
  while (!Serial && (millis() - t0 < 2000)) { delay(10); }

  Serial.println(F("# Simple Sensor Printer starting..."));

  // I2C
  Wire.begin();

  // ----- BMP390 init -----
  if (!bmp.begin_I2C(BMP3XX_I2C_ADDR)) {
    Serial.println(F("# BMP390 init FAILED"));
  } else {
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_16X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_7);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);
    // Establish altitude zero after first valid read
    if (bmp.performReading()) {
      altRef_m = bmp.readAltitude(SEA_LEVEL_HPA); // absolute altitude at startup
      Serial.print(F("# Altitude reference set (m): "));
      Serial.println(altRef_m, 2);
    }
  }

  // ----- BNO08x init -----
  if (bno08x.begin_I2C()) {
    haveBNO = true;
    enableBNOReports();
  } else {
    Serial.println(F("# BNO08x init FAILED (continuing without)"));
  }

  // ----- ADXL375 init -----
  if (!adxl.begin()) {
    Serial.println(F("# ADXL375 init FAILED"));
  } else {
    adxl.setDataRate(ADXL3XX_DATARATE_100_HZ); // 100 Hz is fine for printing
  }

  // Print CSV header
  Serial.println(F("time_ms,temp_C,press_hPa,alt_abs_m,alt_agl_m,yaw_deg,pitch_deg,roll_deg,ax_g,ay_g,az_g,am_g,batt_V"));
}

void loop() {
  const uint32_t now = millis();

  // ---- BMP390 ----
  float temp_C = NAN, press_hPa = NAN, alt_abs_m = NAN, alt_agl_m = NAN;
  if (bmp.performReading()) {
    temp_C    = bmp.temperature;
    press_hPa = bmp.pressure / 100.0f;
    alt_abs_m = bmp.readAltitude(SEA_LEVEL_HPA);
    if (!isnan(altRef_m)) alt_agl_m = alt_abs_m - altRef_m;
  }

  // ---- BNO08x (read any pending events) ----
  if (haveBNO) {
    while (bno08x.getSensorEvent(&bnoVal)) {
      if (bnoVal.sensorId == SH2_ROTATION_VECTOR) {
        // Quaternion is in .un.rotationVector
        float qi = bnoVal.un.rotationVector.i;
        float qj = bnoVal.un.rotationVector.j;
        float qk = bnoVal.un.rotationVector.k;
        float qr = bnoVal.un.rotationVector.real;
        quatToYPR(qr, qi, qj, qk, yaw_deg, pitch_deg, roll_deg);
      }
      // You can also read SH2_ACCELEROMETER / SH2_GYROSCOPE if desired
    }
  }

  // ---- ADXL375 (m/s^2 -> g) ----
  sensors_event_t evt;
  if (adxl.getEvent(&evt)) {
    ax_g = evt.acceleration.x / 9.80665f;
    ay_g = evt.acceleration.y / 9.80665f;
    az_g = evt.acceleration.z / 9.80665f;
    am_g = sqrtf(ax_g*ax_g + ay_g*ay_g + az_g*az_g);
  }

  // ---- Battery (Feather M0 A7 uses /2 divider) ----
  analogReference(AR_DEFAULT);
  uint16_t raw = analogRead(VBATPIN);
  batt_V = (raw * (3.3f / 1024.0f)) * 2.0f;

  // ---- Print at ~10 Hz ----
  if (now - t_lastPrint >= PRINT_PERIOD_MS) {
    t_lastPrint = now;
    // CSV line
    Serial.print(now);                 Serial.print(',');
    // temp_C
    if (isnan(temp_C)) Serial.print("NaN"); else { Serial.print(temp_C, 2); }
    Serial.print(',');
    // press_hPa
    if (isnan(press_hPa)) Serial.print("NaN"); else { Serial.print(press_hPa, 2); }
    Serial.print(',');
    // alt_abs_m
    if (isnan(alt_abs_m)) Serial.print("NaN"); else { Serial.print(alt_abs_m, 2); }
    Serial.print(',');
    // alt_agl_m
    if (isnan(alt_agl_m)) Serial.print("NaN"); else { Serial.print(alt_agl_m, 2); }
    Serial.print(',');
    // yaw, pitch, roll
    if (isnan(yaw_deg))   Serial.print("NaN"); else { Serial.print(yaw_deg, 2); }
    Serial.print(',');
    if (isnan(pitch_deg)) Serial.print("NaN"); else { Serial.print(pitch_deg, 2); }
    Serial.print(',');
    if (isnan(roll_deg))  Serial.print("NaN"); else { Serial.print(roll_deg, 2); }
    Serial.print(',');
    // ax, ay, az, am
    if (isnan(ax_g)) Serial.print("NaN"); else { Serial.print(ax_g, 3); }
    Serial.print(',');
    if (isnan(ay_g)) Serial.print("NaN"); else { Serial.print(ay_g, 3); }
    Serial.print(',');
    if (isnan(az_g)) Serial.print("NaN"); else { Serial.print(az_g, 3); }
    Serial.print(',');
    if (isnan(am_g)) Serial.print("NaN"); else { Serial.print(am_g, 3); }
    Serial.print(',');
    // batt_V
    if (isnan(batt_V)) Serial.print("NaN"); else { Serial.print(batt_V, 2); }
    Serial.println();
  }
}
