void setup() {
  Serial.begin(115200);  // USB serial monitor
  Serial1.begin(115200);   // RX/TX pins from other board
}

void loop() {
  if (Serial1.available()) {
    char c = Serial1.read();
    Serial.write(c);     // Echo to USB monitor
  }
}
