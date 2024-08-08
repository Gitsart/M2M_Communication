void setup() {
  pinMode(19, INPUT);
  pinMode(22, OUTPUT);
  Serial.begin(115200);

}

void loop() {
  int rUn = digitalRead(19);
  if (rUn == 1) {
    digitalWrite(22, HIGH);
    Serial.println("high");
  }
  else if (rUn == 0) {
    digitalWrite(22, LOW);
    Serial.println("low");
  }

}
