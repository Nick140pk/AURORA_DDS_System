#include <SPI.h>
#include <RH_RF95.h>

// Feather M0 RFM95 pins
#define RFM95_CS   8
#define RFM95_RST  4
#define RFM95_INT  3
#define RF95_FREQ  915.0

#define DDS_BAUD   115200
#define APPEND_NEWLINE 1
#define LED_PIN 13

RH_RF95 rf95(RFM95_CS, RFM95_INT);

bool lora_ok = false;

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && (millis() - t0) < 5000) { /* wait up to 5s */ }
  Serial.println("\n[Boot] Avionics LoRa RX → Serial1 forwarder starting...");

  // DDS UART (pins 0/1)
  Serial1.begin(DDS_BAUD);
  Serial.print("[Boot] Serial1 (DDS) @ "); Serial.println(DDS_BAUD);

  // Reset LoRa
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH); delay(10);
  digitalWrite(RFM95_RST, LOW);  delay(10);
  digitalWrite(RFM95_RST, HIGH); delay(10);

  if (!rf95.init()) {
    Serial.println("[ERROR] LoRa init FAILED");
  } else if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("[ERROR] LoRa setFrequency FAILED");
  } else {
    rf95.setModemConfig(RH_RF95::Bw500Cr45Sf128); // BW=500 kHz, CR=4/5, SF7
    rf95.setTxPower(13, false);
    lora_ok = true;
    Serial.print("[OK] LoRa listening @ "); Serial.println(RF95_FREQ, 1);
  }

  Serial.println("[Boot] Ready. Waiting for packets...");
}

void loop() {
// 1 Hz heartbeat so you always see *something*
static unsigned long lastHb = 0;
static bool ledState = false;
if (millis() - lastHb >= 1000) {
  lastHb = millis();
  ledState = !ledState;
  digitalWrite(LED_PIN, ledState);
  Serial.print("[HB] alive, lora_ok=");
  Serial.println(lora_ok ? "1" : "0");
}


  if (lora_ok && rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN + 1];
    uint8_t len = sizeof(buf) - 1;

    if (rf95.recv(buf, &len)) {
      // Forward to DDS (binary-safe)
      Serial1.write(buf, len);
#if APPEND_NEWLINE
      Serial1.write('\n');
#endif
      // Debug to USB
      Serial.print("[RX] len="); Serial.print(len);
      Serial.print(" RSSI="); Serial.print(rf95.lastRssi(), 1);
      Serial.print(" SNR=");  Serial.println(rf95.lastSNR(), 1);

      buf[len] = 0;
      Serial.println((char*)buf);
    } else {
      Serial.println("[WARN] LoRa recv failed");
    }
  }

  delay(1);
}
