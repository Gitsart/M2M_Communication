void setup() {
  Serial.begin(115200);
  Serial.println("ESP core");
  Serial.println("*");
  
}

void loop() {
  Serial.print("loop() running on core:");
  Serial.println(xPortGetCoreID());
}
