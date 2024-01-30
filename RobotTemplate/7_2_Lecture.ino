int IRLEDpin = 3; // P3.2 <-> yellow wire
int redLEDpin = 4; // P3.3 <-> green wire

void setup() {
  pinMode(IRLEDpin, OUTPUT);
  pinMode(redLEDpin, OUTPUT);
}

void loop() {
  digitalWrite(IRLEDpin, HIGH);
  digitalWrite(redLEDpin, HIGH);
  delay(5000);
  digitalWrite(IRLEDpin, LOW);
  digitalWrite(redLEDpin, LOW);
  delay(1000);
}
