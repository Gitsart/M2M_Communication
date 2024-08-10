#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP32_AP";
const char* password = "12345678";
const int udpPort = 12345;
WiFiUDP udp;

// Define RX and TX pins for Serial communication with Arduino Mega
#define RXD2 16
#define TXD2 17

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);  // Initialize Serial2 for communication with Mega

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to AP...");
  }
  Serial.println("Connected to AP");

  udp.begin(udpPort);
}

void loop() {
  // Check if data is received via UDP
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    Serial.printf("Received via UDP: %s\n", incomingPacket);

    // Send received UDP data to Mega via Serial2
    Serial2.write(incomingPacket[0]);
    Serial.printf("Sent to Mega: %c\n", incomingPacket[0]);

    // Check if received character is 'B'
    if (incomingPacket[0] == 'B') {
      // Send 'R' back to AP
      udp.beginPacket("192.168.4.1", udpPort);  // Send to AP IP
      udp.write('R');  // Send the character 'R'
      udp.endPacket();
      Serial.println("Sent to AP: R");
    }
  }
}
