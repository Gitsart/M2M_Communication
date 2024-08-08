#include <WiFi.h>
#include <WiFiUdp.h>

#define RXD2 16
#define TXD2 17

const char* ssid = "ESP32_AP";
const char* password = "12345678";
const int udpPort = 12345;
WiFiUDP udp;

void setup() {
  Serial.begin(115200);      // Debugging
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);   // Communication with Arduino Mega

  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  udp.begin(udpPort);
}

void loop() {
  // Check if data is available to read from Serial2
  if (Serial2.available()) {
    char receivedChar = Serial2.read();
    Serial.print("Received from Mega: ");
    Serial.println(receivedChar);

    // Send character to STA via UDP
    udp.beginPacket("192.168.4.2", udpPort);  // Adjust STA IP if needed
    udp.write(receivedChar);
    udp.endPacket();
    Serial.print("Sent to STA: ");
    Serial.println(receivedChar);
  }

  // Check if data is received via UDP
  int packetSize = udp.parsePacket();
  if (packetSize) {
    char incomingPacket[255];
    int len = udp.read(incomingPacket, 255);
    if (len > 0) {
      incomingPacket[len] = 0;
    }
    Serial.printf("Received via UDP: %s\n", incomingPacket);
  }
}
