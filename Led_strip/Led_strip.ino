#define LED_PIN 6
#define LED_COUNT 60 // Replace with the number of LEDs in your strip

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // Turn on the LED strip (assuming it's connected to pin 6)
  digitalWrite(LED_PIN, HIGH);
  delay(1000); // Wait for 1 second

  // Turn off the LED strip
  digitalWrite(LED_PIN, LOW);
  delay(1000); // Wait for 1 second
}
