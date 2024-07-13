#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// Pins:
const int HX711_dout = 14; // MCU > HX711 dout pin
const int HX711_sck = 12; // MCU > HX711 sck pin
const int LED_PIN = 27;    // Pin for LED

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
float userWeightGrams = 0.0;

void setup() {
  Serial.begin(57600); delay(10);
  Serial.println();
  Serial.println("Starting...");

  pinMode(LED_PIN, OUTPUT);  // Set LED pin as output
  digitalWrite(LED_PIN, LOW); // Turn off LED initially

  LoadCell.begin();
  // LoadCell.setReverseOutput(); // Uncomment to turn a negative output value to positive
  unsigned long stabilizingtime = 2000; // Precision right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // Set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0); // User set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate(); // Start calibration procedure
}

void loop() {
  // Receive command from serial terminal
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay(); // Tare
    else if (inByte == 'r') calibrate(); // Calibrate
    else if (inByte == 'c') changeSavedCalFactor(); // Edit calibration value manually
    else if (inByte == 'u') setUserWeight(); // Set user weight
    else if (inByte == 'm') measureWeight(); // Measure weight
  }

  // Check if last tare operation is complete
  if (LoadCell.getTareStatus() == true) {
    Serial.println("Tare complete");
  }
}

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place the load cell on a level stable surface.");
  Serial.println("Remove any load applied to the load cell.");
  Serial.println("Send 't' from serial monitor to set the tare offset.");

  boolean _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 't') LoadCell.tareNoDelay();
    }
    if (LoadCell.getTareStatus() == true) {
      Serial.println("Tare complete");
      _resume = true;
    }
  }

  Serial.println("Now, place your known mass on the load cell.");
  Serial.println("Then send the weight of this mass (i.e. 1000) from serial monitor (grams).");

  float known_mass_grams = 0;
  _resume = false;
  while (_resume == false) {
    LoadCell.update();
    if (Serial.available() > 0) {
      known_mass_grams = Serial.parseFloat();
      if (known_mass_grams != 0) {
        Serial.print("Known mass is: ");
        Serial.print(known_mass_grams);
        Serial.println(" grams");
        _resume = true;
      }
    }
  }

  LoadCell.refreshDataSet(); // Refresh the dataset to be sure that the known mass is measured correctly
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass_grams); // Get the new calibration value

  Serial.print("New calibration value has been set to: ");
  Serial.print(newCalibrationValue);
  Serial.println(", use this as calibration value (calFactor) in your project sketch.");
  Serial.print("Save this value to EEPROM address ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");

  _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }

  Serial.println("End calibration");
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("***");

  setUserWeight(); // Get user weight after calibration
}

void setUserWeight() {
  Serial.println("Enter your weight (i.e. 700) and send it from serial monitor (grams):");

  boolean _resume = false;
  while (_resume == false) {
    if (Serial.available() > 0) {
      userWeightGrams = Serial.parseFloat();
      if (userWeightGrams != 0) {
        Serial.print("User weight set to: ");
        Serial.print(userWeightGrams);
        Serial.println(" grams");
        _resume = true;
      }
    }
  }
}

void measureWeight() {
  Serial.println("Measuring weight...");
  LoadCell.update(); // Ensure the LoadCell is updated before taking the reading

  // Wait for a stable reading
  while (LoadCell.update());

  float initialWeightGrams = LoadCell.getData(); // Get initial weight
  float currentWeightGrams = initialWeightGrams;
  Serial.print("Initial weight: ");
  Serial.print(initialWeightGrams);
  Serial.println(" grams");

  while (1) {
    LoadCell.update(); // Update the LoadCell data

    // Check if the weight has changed significantly (you can adjust the threshold as needed)
    if (abs(currentWeightGrams - LoadCell.getData()) >= 10) { // 10 grams threshold
      currentWeightGrams = LoadCell.getData(); // Update current weight
      Serial.print("Current weight: ");
      Serial.print(currentWeightGrams);
      Serial.println(" grams");

      // Check if weight exceeds user-specified weight and control LED
      if (currentWeightGrams > userWeightGrams*20/100) {
        digitalWrite(LED_PIN, HIGH); // Turn on LED
        Serial.println("Weight exceeds user weight. LED turned on.");
      } else {
        digitalWrite(LED_PIN, LOW);  // Turn off LED
        Serial.println("Weight is within limit. LED turned off.");
      }
    }

    // Add a small delay to avoid overwhelming the serial output
    delay(100);
  }
}

void changeSavedCalFactor() {
  float oldCalibrationValue = LoadCell.getCalFactor();
  boolean _resume = false;
  Serial.println("***");
  Serial.print("Current value is: ");
  Serial.println(oldCalibrationValue);
  Serial.println("Now, send the new value from serial monitor, i.e. 696.0");
  float newCalibrationValue;
  while (_resume == false) {
    if (Serial.available() > 0) {
      newCalibrationValue = Serial.parseFloat();
      if (newCalibrationValue != 0) {
        Serial.print("New calibration value is: ");
        Serial.println(newCalibrationValue);
        LoadCell.setCalFactor(newCalibrationValue);
        _resume = true;
      }
    }
  }
  _resume = false;
  Serial.print("Save this value to EEPROM address ");
  Serial.print(calVal_eepromAdress);
  Serial.println("? y/n");
  while (_resume == false) {
    if (Serial.available() > 0) {
      char inByte = Serial.read();
      if (inByte == 'y') {
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.begin(512);
#endif
        EEPROM.put(calVal_eepromAdress, newCalibrationValue);
#if defined(ESP8266)|| defined(ESP32)
        EEPROM.commit();
#endif
        EEPROM.get(calVal_eepromAdress, newCalibrationValue);
        Serial.print("Value ");
        Serial.print(newCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        _resume = true;
      }
      else if (inByte == 'n') {
        Serial.println("Value not saved to EEPROM");
        _resume = true;
      }
    }
  }
  Serial.println("End change calibration value");
  Serial.println("***");
}
