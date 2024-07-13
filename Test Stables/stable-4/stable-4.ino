#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <HX711_ADC.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif

// WiFi credentials
const char* ssid = "12345678";
const char* password = "12345678";

// Server and GPS objects
WebServer server(80);
TinyGPSPlus gps; // Object to handle GPS parsing
HardwareSerial ss(1); // Use Serial1 for GPS

// RTC object
RTC_DS3231 rtc;

// DS18B20 sensor
#define ONE_WIRE_BUS 13
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// BH1750 light sensor
BH1750 lightMeter;

//HX711 PINS
const int HX711_dout = 14; // MCU > HX711 dout pin
const int HX711_sck = 12; // MCU > HX711 sck pin
const int LED_PIN = 27;    // Pin for LED

// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
float userWeightGrams = 0.0;

// Sensor data
float temperature = 0.0;
float weight = 70.0; // Placeholder for weight sensor data
float lux = 0.0; // Light level in lux
String gpsData = "6.7951, 79.9009"; // Initial example GPS coordinates
String lockStatus = "unlocked"; // Example lock status

unsigned long previousMillis = 0; // Variable to store the last time data was printed
const long interval = 5000; // Interval between data prints (in milliseconds)

const int lightPin = 26; // Digital pin for the LED
bool lightStatus = false; // Light status
const float lightThreshold = 10.0; // Light intensity threshold for LED control

TwoWire I2C_RTC = TwoWire(0);
TwoWire I2C_BH1750 = TwoWire(1);

void setup() {
  Serial.begin(115200);
  ss.begin(9600, SERIAL_8N1, 16, 17); // NEO-7M RX, TX
  pinMode(lightPin, OUTPUT); // Set BH1750 LED pin as output
  digitalWrite(lightPin, LOW); // Ensure BH1750 LED is off initially

  pinMode(LED_PIN, OUTPUT);  // Set HX711 LED pin as output
  digitalWrite(LED_PIN, LOW); // Turn off HX711 LED initially

  I2C_RTC.begin(15, 0, 100000); // SDA = 15, SCL = 0
  I2C_BH1750.begin(21, 22, 100000); // SDA = 21, SCL = 22

  if (!rtc.begin(&I2C_RTC)) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  sensors.begin();

  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2C_BH1750)) {
    Serial.println(F("BH1750 initialized"));
  } else {
    Serial.println(F("Error initializing BH1750"));
  }

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  LoadCell.begin();
  unsigned long stabilizingtime = 2000; 
  boolean _tare = true; 
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  }
  else {
    LoadCell.setCalFactor(1.0);
    Serial.println("Startup is complete");
  }
  while (!LoadCell.update());
  calibrate();

  Serial.println();
  Serial.print("Connected to the WiFi network with IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot);
  server.on("/data", handleData);
  server.on("/update", handleUpdate);
  server.on("/light", handleLight);

  server.begin();
  Serial.println("HTTP server started");

  Serial.println("Enter new data through the Serial Monitor:");
}

void handleRoot() {
  String html = "<html><body>";
  html += "<h1>ESP32 Data Dashboard</h1>";
  html += "<p>Temperature: " + String(temperature) + " °C</p>";
  html += "<p>Weight: " + String(weight) + "</p>";
  html += "<p>Light: " + String(lux) + " lx</p>";
  html += "<p>GPS: " + gpsData + "</p>";
  html += "<p>Lock Status: " + lockStatus + "</p>";
  html += "<p>Light Status: " + String(lightStatus ? "on" : "off") + "</p>";
  html += "<p>RTC DateTime: " + getRTCDateTime() + "</p>";
  html += "</body></html>";
  server.send(200, "text/html", html);
}

void handleData() {
  String json = "{\"temperature\": " + String(temperature) + ",";
  json += "\"weight\": " + String(weight) + ",";
  json += "\"light\": " + String(lux) + ",";
  json += "\"gps\": \"" + gpsData + "\",";
  json += "\"lockStatus\": \"" + lockStatus + "\",";
  json += "\"lightStatus\": \"" + String(lightStatus ? "on" : "off") + "\",";
  json += "\"rtcDateTime\": \"" + getRTCDateTime() + "\"}";
  server.send(200, "application/json", json);
}

void handleUpdate() {
  if (server.method() == HTTP_POST) {
    String body = server.arg("plain");
    body.trim();

    Serial.println("User Weight Updated:");
    Serial.println(body);

    if (body.startsWith("temp:")) {
      temperature = body.substring(5).toFloat();
      Serial.println("Temperature updated to: " + String(temperature));
    } else if (body.startsWith("weight:")) {
      weight = body.substring(7).toFloat();
      Serial.println("Weight updated to: " + String(weight));
    } else if (body.startsWith("gps:")) {
      gpsData = body.substring(4);
      Serial.println("GPS data updated to: " + gpsData);
    } else if (body.startsWith("lock:")) {
      lockStatus = body.substring(5);
      Serial.println("Lock status updated to: " + lockStatus);
    }

    server.send(200, "text/plain", "Data updated");
  } else {
    server.send(405, "text/plain", "Method Not Allowed");
  }
}

void handleLight() {
  if (server.hasArg("state")) {
    String state = server.arg("state");
    Serial.println("Received light control request with state: " + state);
    if (state == "on") {
      lightStatus = true;
      Serial.println("Light turned on");
    } else if (state == "off") {
      lightStatus = false;
      Serial.println("Light turned off");
    }
    server.send(200, "text/plain", "Light toggled");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
}

void loop() {
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    printSensorData();
  }

  server.handleClient();

  updateSensorData();
  readGPSData();
  readTemperatureData();

  if (lightStatus) {
    readLightData();
  }

  readWeightSensor();
}

void readLightData() {
  lux = lightMeter.readLightLevel();
}

void printSensorData() {
  Serial.println("--------------------");
  Serial.println("Sensor Data:");
  Serial.print("Temperature: ");
  Serial.println(temperature);
  Serial.print("Weight: ");
  Serial.println(weight);
  Serial.print("Light: ");
  Serial.println(lux);
  Serial.print("GPS: ");
  Serial.println(gpsData);
  Serial.print("Lock Status: ");
  Serial.println(lockStatus);
  Serial.print("Light Status: ");
  Serial.println(lightStatus ? "on" : "off");
  Serial.print("RTC DateTime: ");
  Serial.println(getRTCDateTime());
  Serial.println("--------------------");
}

void updateSensorData() {
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any trailing newline characters

    Serial.println("Received serial input: " + input);

    if (input.startsWith("temp:")) {
      temperature = input.substring(5).toFloat();
      Serial.println("Temperature updated to: " + String(temperature));
    } else if (input.startsWith("weight:")) {
      weight = input.substring(7).toFloat();
      Serial.println("Weight updated to: " + String(weight));
     } else if (input.startsWith("gps:")) {
      gpsData = input.substring(4);
      Serial.println("GPS data updated to: " + gpsData);
    } else if (input.startsWith("lock:")) {
      lockStatus = input.substring(5);
      Serial.println("Lock status updated to: " + lockStatus);
    }
  }
}

void readGPSData() {
  while (ss.available() > 0) {
    if (gps.encode(ss.read())) {
      if (gps.location.isValid()) {
        float latitude = gps.location.lat();
        float longitude = gps.location.lng();
        gpsData = String(latitude, 6) + "," + String(longitude, 6);
      } else {
        gpsData = "Invalid GPS data";
      }
    }
  }
}

void readTemperatureData() {
  sensors.requestTemperatures();
  temperature = sensors.getTempCByIndex(0);
}

void readWeightSensor() {
  if (Serial.available() > 0) {
    char inByte = Serial.read();
    if (inByte == 't') LoadCell.tareNoDelay();
    else if (inByte == 'r') calibrate();
    else if (inByte == 'c') changeSavedCalFactor();
    else if (inByte == 'u') setUserWeight();
    else if (inByte == 'm') measureWeight();
  }

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

  LoadCell.refreshDataSet();
  float newCalibrationValue = LoadCell.getNewCalibration(known_mass_grams);

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

  setUserWeight();
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
  LoadCell.update();

  while (LoadCell.update());

  float initialWeightGrams = LoadCell.getData();
  float currentWeightGrams = initialWeightGrams;
  Serial.print("Initial weight: ");
  Serial.print(initialWeightGrams);
  Serial.println(" grams");

  while (1) {
    LoadCell.update();

    if (abs(currentWeightGrams - LoadCell.getData()) >= 10) {
      currentWeightGrams = LoadCell.getData();
      Serial.print("Current weight: ");
      Serial.print(currentWeightGrams);
      Serial.println(" grams");

      if (currentWeightGrams > userWeightGrams) {
        digitalWrite(LED_PIN, HIGH);
        Serial.println("Weight exceeds user weight. LED turned on.");
      } else {
        digitalWrite(LED_PIN, LOW);
        Serial.println("Weight is within limit. LED turned off.");
      }
    }

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
        LoadCell.setCalFactor(oldCalibrationValue);
        _resume = true;
      }
    }
  }
  Serial.println("***");
  Serial.println("To re-calibrate, send 'r' from serial monitor.");
  Serial.println("For manual edit of the calibration value, send 'c' from serial monitor.");
  Serial.println("***");
}

String getRTCDateTime() {
  DateTime now = rtc.now();
  String dateTime = String(now.year()) + "-" +
                    String(now.month()) + "-" +
                    String(now.day()) + " " +
                    String(now.hour()) + ":" +
                    String(now.minute()) + ":" +
                    String(now.second());
  return dateTime;
}


