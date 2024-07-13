#include <Wire.h>
#include <BH1750.h>
#include <FastLED.h>

// Define the number of LEDs in the strip
#define NUM_LEDS 7

// Define the data pin for the LED strip
#define DATA_PIN 3

// Create an instance of the BH1750 sensor
BH1750 lightMeter;

// Create an array to hold the LED data
CRGB leds[NUM_LEDS];

// Define the RGB color for the LED strip (0-255 for each channel)
uint8_t redValue = 0;   // Red channel value (0-255)
uint8_t greenValue = 128; // Green channel value (0-255)
uint8_t blueValue = 128;    // Blue channel value (0-255)

void setup() {
  Serial.begin(9600);

  // Initialize the FastLED library
  FastLED.addLeds<WS2812B, DATA_PIN, GRB>(leds, NUM_LEDS);

  // Initialize the BH1750 sensor
  Wire.begin();
  lightMeter.begin();
}

void loop() {
  // Read the lux value from the BH1750 sensor
  float lux = lightMeter.readLightLevel();

  // Print the lux value to the serial monitor
  Serial.print("Lux: ");
  Serial.println(lux);

  // If the lux value is lower than 500, turn on the LED strip with the specified RGB color
  if (lux < 20) {
    fill_solid(leds, NUM_LEDS, CRGB(redValue, greenValue, blueValue));
    FastLED.show();
  } else {
    // Otherwise, turn off the LED strip
    fill_solid(leds, NUM_LEDS, CRGB::Black);
    FastLED.show();
  }

  delay(100); // Add a small delay to avoid flooding the serial monitor
}