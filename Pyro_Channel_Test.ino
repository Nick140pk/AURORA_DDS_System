#include <Arduino.h>

// ===== Pins (your usual Feather M0 setup) =====
constexpr uint8_t PIN_DROGUE_1 = 5;
constexpr uint8_t PIN_DROGUE_2 = 6;
constexpr uint8_t PIN_MAIN_1   = 9;
constexpr uint8_t PIN_MAIN_2   = 10;
constexpr uint8_t PIN_BUZZER   = 11;

// ===== Safety: set false to simulate without driving pyro outputs =====
const bool ARMED = true;

// ===== Timing (all in ms) =====
constexpr uint16_t BEEP_SHORT_MS            = 120;   // short beep for ticks
constexpr uint16_t BEEP_PREFIRE_MS          = 2000;  // 2s warning beep before each fire
constexpr uint16_t PYRO_HOLD_MS             = 2000;  // keep channel HIGH for 2s
constexpr uint16_t GAP_DROGUE_CHANNELS_MS   = 1500;  // 1.5s between drogue 1 & 2
constexpr uint16_t GAP_MAIN_CHANNELS_MS     = 1500;  // 1.5s between main 1 & 2
constexpr uint16_t WAIT_BEFORE_MAIN_MS      = 5000;  // 5s between drogue 2 and main 1
constexpr uint8_t  COUNTDOWN_START          = 10;    // 10..1
constexpr uint8_t  RAPID_BEEP_COUNT         = 10;    // finish flourish

// ===== Beeper helpers =====
void beep(uint16_t ms) {
  digitalWrite(PIN_BUZZER, HIGH);
  delay(ms);
  digitalWrite(PIN_BUZZER, LOW);
}

void rapidBeep(uint8_t n) {
  for (uint8_t i = 0; i < n; i++) {
    beep(80);
    delay(80);
  }
}

// Keeps a 1 Hz rhythm: short beep every second for N seconds
void beepEverySecond(uint8_t seconds, const char* label = nullptr) {
  for (int i = seconds; i >= 1; --i) {
    if (label) {
      Serial.print(label);
      Serial.print(" ");
      Serial.println(i);
    } else {
      Serial.print("T-");
      Serial.println(i);
    }
    // short beep + remainder of the second
    beep(BEEP_SHORT_MS);
    // avoid negative in case someone tweaks BEEP_SHORT_MS
    uint16_t rest = (BEEP_SHORT_MS < 1000) ? (1000 - BEEP_SHORT_MS) : 0;
    delay(rest);
  }
}

// ===== Pyro helpers =====
void setSafeLowAllPyros() {
  digitalWrite(PIN_DROGUE_1, LOW);
  digitalWrite(PIN_DROGUE_2, LOW);
  digitalWrite(PIN_MAIN_1,   LOW);
  digitalWrite(PIN_MAIN_2,   LOW);
}

// 2s warning beep, then hold pin HIGH for 2s, then LOW
void fireChannel(uint8_t pin, const char* name) {
  Serial.print("Preparing to FIRE ");
  Serial.println(name);

  // 2s pre-fire warning beep
  beep(BEEP_PREFIRE_MS);
  delay(100); // tiny pause to end tone cleanly

  if (ARMED) {
    Serial.print("FIRING ");
    Serial.println(name);
    digitalWrite(pin, HIGH);
    delay(PYRO_HOLD_MS);
    digitalWrite(pin, LOW);
    Serial.print("DONE ");
    Serial.println(name);
  } else {
    Serial.print("(SIM) Would fire ");
    Serial.println(name);
    delay(PYRO_HOLD_MS);
  }
}

void setup() {
  // Pins
  pinMode(PIN_DROGUE_1, OUTPUT);
  pinMode(PIN_DROGUE_2, OUTPUT);
  pinMode(PIN_MAIN_1,   OUTPUT);
  pinMode(PIN_MAIN_2,   OUTPUT);
  pinMode(PIN_BUZZER,   OUTPUT);
  setSafeLowAllPyros();
  digitalWrite(PIN_BUZZER, LOW);

  // Serial
  Serial.begin(115200);
  delay(50);
  Serial.println();
  Serial.println("=== Pyro Sequence Simple Sketch ===");
  Serial.print("ARMED: ");
  Serial.println(ARMED ? "TRUE (LIVE)" : "FALSE (SIMULATION)");

  // Power-on two short beeps
  beep(BEEP_SHORT_MS);
  delay(150);
  beep(BEEP_SHORT_MS);
  delay(300);

  // Countdown 10..1 with a short beep each second
  Serial.println("Starting countdown...");
  beepEverySecond(COUNTDOWN_START); // prints T-10..T-1

  // Drogue sequence
  fireChannel(PIN_DROGUE_1, "DROGUE_1");
  delay(GAP_DROGUE_CHANNELS_MS);
  fireChannel(PIN_DROGUE_2, "DROGUE_2");

  // 5 seconds wait with a beep at each second, then mains
  Serial.println("Waiting before MAIN deployment...");
  beepEverySecond(WAIT_BEFORE_MAIN_MS / 1000, "T-");

  // Main sequence
  fireChannel(PIN_MAIN_1, "MAIN_1");
  delay(GAP_MAIN_CHANNELS_MS);
  fireChannel(PIN_MAIN_2, "MAIN_2");

  // Finish flourish: rapid beeps
  Serial.println("Sequence complete.");
  rapidBeep(RAPID_BEEP_COUNT);

  // Ensure all pyros are LOW at the end
  setSafeLowAllPyros();
}

void loop() {
  // Do nothing after the one-shot sequence.
}
