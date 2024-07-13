int ledPin = 8;      // LED connected to digital pin 8
int switchPin = 7;   // Rocker switch connected to digital pin 7
int switchState = 0; // Variable for reading the switch status

void setup() {
  pinMode(ledPin, OUTPUT);       // Set LED pin as an output
  pinMode(switchPin, INPUT_PULLUP); // Set switch pin as an input with internal pull-up resistor
}

void loop() {
  switchState = digitalRead(switchPin); // Read the state of the switch

  if (switchState == LOW) { // Switch is pressed (connected to ground)
    digitalWrite(ledPin, HIGH);  // Turn the LED on
  } else { // Switch is not pressed (internal pull-up makes it HIGH)
    digitalWrite(ledPin, LOW);   // Turn the LED off
  }
}
