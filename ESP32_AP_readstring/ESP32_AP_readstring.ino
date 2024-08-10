#include <WiFi.h>
#include <WiFiUdp.h>

const char* ssid = "ESP32_AP";
const char* password = "12345678";
const int udpPort = 12345;
const int buttonPin = 19;
WiFiUDP udp;

bool lastButtonState = HIGH;

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);
  WiFi.softAP(ssid, password);
  Serial.println("Access Point started");

  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);

  udp.begin(udpPort);
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  if (buttonState != lastButtonState) {
    lastButtonState = buttonState;
    if (buttonState == HIGH) {
      Serial.println("Button state HIGH on AP");
      udp.beginPacket("192.168.4.2", udpPort);  // Send to STA IP (adjust if needed)
      udp.print("Wait, I am running");
      udp.endPacket();
    } else {
      Serial.println("Button state LOW on AP");
      udp.beginPacket("192.168.4.2", udpPort);  // Send to STA IP (adjust if needed)
      udp.print("Stopped, You can Run");
      udp.endPacket();
    }
    delay(100);  // Debounce delay
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
