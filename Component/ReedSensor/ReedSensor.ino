#include <WiFi.h>

// WiFi credentials
const char* ssid = "12345678";
const char* password = "12345678";

// Define the GPIO pin where the reed switch is connected
const int reedSwitchPin = 4;
const int ledPin = 27;

// Define the state variable
enum State { OPEN, CLOSED };
State currentState;

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);

  // Set the reed switch pin as an input with an internal pull-up resistor
  pinMode(reedSwitchPin, INPUT_PULLUP);

  // Set the LED pin as an output
  pinMode(ledPin, OUTPUT);
  // digitalWrite(ledPin, LOW); // Ensure LED is off initially

  // Initialize the state
  currentState = OPEN;

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected to WiFi");
}

void loop() {
  // Read the state of the reed switch
  int reedState = digitalRead(reedSwitchPin);

  // Update the state based on the reed switch status
  if (reedState == LOW && currentState != CLOSED) {
    currentState = CLOSED;
    Serial.println("State changed to: CLOSED");
  } else if (reedState == HIGH && currentState != OPEN) {
    currentState = OPEN;
    Serial.println("State changed to: OPEN");
  }

  // Check WiFi connection status and reed switch state to control the LED
  if (WiFi.status() != WL_CONNECTED && currentState == OPEN) {
    // digitalWrite(ledPin, HIGH);
    tone(ledPin, 1000);  // Turn LED on
  } else {
    // digitalWrite(ledPin, LOW); 
    noTone(ledPin);  // Turn LED off
  }

  // Add a small delay to avoid flooding the serial monitor
  delay(500);
}