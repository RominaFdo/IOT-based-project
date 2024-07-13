// BH1750 to ESP32:

// BH1750 VCC to ESP32 3.3V
// BH1750 GND to ESP32 GND
// BH1750 SDA to ESP32 GPIO 21 (SDA)
// BH1750 SCL to ESP32 GPIO 22 (SCL)

// LED to ESP32:

// LED Anode (+) to ESP32 GPIO 18 (via a current-limiting resistor, typically 1kΩ).
// LED Cathode (-) to ESP32 GND.

#include <Wire.h>
#include <BH1750.h>

// Create an instance of the BH1750 object
BH1750 lightMeter;

// Define the GPIO pin for the LED
const int ledPin = 26;

// Define the light intensity threshold
const float lightThreshold = 10.0; // Adjust this value as needed

void setup() {
  // Start the Serial Monitor
  Serial.begin(115200);

  // Initialize I2C communication as MASTER
  Wire.begin();

  // Initialize the BH1750 sensor
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE)) {
    Serial.println(F("BH1750 Advanced begin"));
  } else {
    Serial.println(F("Error initialising BH1750"));
  }

  // Initialize the LED pin as an output
  pinMode(ledPin, OUTPUT);

  // Ensure the LED is off initially
  digitalWrite(ledPin, LOW);
}

void loop() {
  // Read light level in lux
  float lux = lightMeter.readLightLevel() ;
  
  // Print the light level
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  
  // Control the LED based on the light intensity
  if (lux < lightThreshold) {
    // Turn on the LED if light level is below the threshold
    digitalWrite(ledPin, HIGH);
  } else {
    // Turn off the LED if light level is above the threshold
    digitalWrite(ledPin, LOW);
  }
  
  // Wait for a bit before the next measurement
  delay(1500);
}