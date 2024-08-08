#define RXD2 16
#define TXD2 17

void setup() {
  // Initialize Serial Monitor for debugging
  Serial.begin(115200);
  // Initialize Serial2 for UART communication with Mega
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("ESP32 Ready");
}

void loop() {
  // Check if data is available to read from Serial2
  if (Serial2.available()) {
    while (Serial2.available() > 0) {
      char receivedChar = Serial2.read();
      Serial.print("Received from Mega: ");
      Serial.println(receivedChar);
    }
  }
}
