#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// Pins
const int HX711_dout = 14; // mcu > HX711 dout pin
const int HX711_sck = 12; // mcu > HX711 sck pin
const int buzzerPin = 27; // Buzzer signal pin

// HX711 constructor
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
float userWeightGrams = 0; // Variable to store user weight

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  pinMode(buzzerPin, OUTPUT);

  LoadCell.begin();
  float calibrationValue;
#if defined(ESP8266)|| defined(ESP32)
  EEPROM.begin(512);
#endif
  EEPROM.get(calVal_eepromAdress, calibrationValue);

  unsigned long stabilizingtime = 2000;
  boolean _tare = true;
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(calibrationValue);
    Serial.println("Startup is complete");
  }

  Serial.println("Enter user weight in grams:");
  while (userWeightGrams == 0) {
    if (Serial.available() > 0) {
      userWeightGrams = Serial.parseFloat();
      if (userWeightGrams > 0) {
        Serial.print("User weight set to: ");
        Serial.print(userWeightGrams);
        Serial.println(" grams");
      } else {
        Serial.println("Invalid weight. Please enter a positive number.");
      }
    }
  }
  
  Serial.println("System ready. Place items on the scale.");
}

void loop() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;

  if (LoadCell.update()) newDataReady = true;

  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      float currentWeightGrams = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(currentWeightGrams);

      // Check weight condition
      if (currentWeightGrams > userWeightGrams * 20 / 100) {
        tone(buzzerPin, 2000);  // Turn on buzzer at 1kHz
        Serial.println("Weight exceeds user weight. Buzzer turned on.");
      } else {
        noTone(buzzerPin);  // Turn off buzzer
        Serial.println("Weight is within limit. Buzzer turned off.");
      }

      newDataReady = 0;
      t = millis();
    }
  }

  // Receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
  }

  // Check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}