#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP32_AP";
const char* password = "12345678";
const int udpPort = 12345;
const int buttonPin = 19;
WiFiUDP udp;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to AP...");
  }
  Serial.println("Connected to AP");

  udp.begin(udpPort);
}

void loop() {
  if (digitalRead(buttonPin) == LOW) {
    Serial.println("Button pressed on STA");
    udp.beginPacket("192.168.4.1", udpPort);  // Send to AP IP
    udp.write('B');
    udp.endPacket();
    delay(1000);  // Debounce delay
  }

  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    Serial.printf("Received: %s\n", incomingPacket);
  }
}
