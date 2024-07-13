#include <Wire.h>
#include <BH1750.h>

BH1750 lightMeter;
const int ledPin = 9; // Pin connected to the MOSFET gate or transistor base
const float thresholdLux = 10; // Threshold for turning on the LED strips

void setup() {
  Serial.begin(9600);
  Wire.begin();
  lightMeter.begin();
  pinMode(ledPin, OUTPUT);
}

void loop() {
  float lux = lightMeter.readLightLevel();
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");

  if (lux < thresholdLux) {
    digitalWrite(ledPin, HIGH); // Turn on the LED strips
  } else {
    digitalWrite(ledPin, LOW); // Turn off the LED strips
  }

  delay(1000); // Wait for 1 second before taking another reading
}
