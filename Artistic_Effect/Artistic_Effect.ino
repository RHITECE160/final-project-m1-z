#define redLED 4 // P3.3
#define greenLED 18 // P3.0
#define blueLED 38 // P2.4
int delay_time = 400; // The time in milliseconds between LED flashes.

void setup() { // Sets up the LEDs and their pins as outputs and then engages the serial monitor, default delay time, and asks the user what delay they want.
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(blueLED, OUTPUT);
  delay(delay_time);
}

void loop() { // The pattern of the LEDs. 
  digitalWrite(redLED, HIGH);
  digitalWrite(greenLED, LOW);
  digitalWrite(blueLED, LOW);
  delay(delay_time);
  digitalWrite(greenLED, HIGH);
  delay(delay_time);
  digitalWrite(redLED, LOW);
  delay(delay_time);
  digitalWrite(blueLED, HIGH);
  delay(delay_time);
  digitalWrite(greenLED, LOW);
  delay(delay_time);
  digitalWrite(redLED, HIGH);
  delay(delay_time);
  digitalWrite(greenLED, HIGH);
  digitalWrite(blueLED, HIGH);
  delay(delay_time);
}