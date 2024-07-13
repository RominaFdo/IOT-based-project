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

// HX711 sensor
const int HX711_dout = 14; // MCU > HX711 dout pin
const int HX711_sck = 12; // MCU > HX711 sck pin
HX711_ADC LoadCell(HX711_dout, HX711_sck);
const int calVal_eepromAdress = 0;
float userWeightGrams = 0.0;
float weight = 0.0; // Weight sensor data
unsigned long t = 0; // Timer for serial print interval

// Sensor data
float temperature = 0.0;
float lux = 0.0; // Light level in lux
String gpsData = "6.7951, 79.9009"; // Initial example GPS coordinates
String lockStatus = "unlocked"; // Example lock status

unsigned long previousMillis = 0; // Variable to store the last time data was printed
const long interval = 5000; // Interval between data prints (in milliseconds)

const int lightPin = 26; // Digital pin for the LED
bool lightStatus = false; // Light status
const float lightThreshold = 10.0; // Light intensity threshold for LED control

// Two separate I2C buses
TwoWire I2C_RTC = TwoWire(0);
TwoWire I2C_BH1750 = TwoWire(1);

void setup() {
  Serial.begin(115200);
  ss.begin(9600, SERIAL_8N1, 16, 17); // NEO-7M RX, TX
  pinMode(lightPin, OUTPUT); // Set LED pin as output
  digitalWrite(lightPin, LOW); // Ensure LED is off initially

  // Initialize I2C for RTC
  I2C_RTC.begin(15, 0, 100000); // SDA = 15, SCL = 0

  // Initialize I2C for BH1750
  I2C_BH1750.begin(21, 22, 100000); // SDA = 21, SCL = 22

  // Initialize RTC
  if (!rtc.begin(&I2C_RTC)) {
    Serial.println("Couldn't find RTC");
    while (1);
  }

  // If RTC lost power, set the time to the compile time
  if (rtc.lostPower()) {
    Serial.println("RTC lost power, setting the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize DS18B20 temperature sensor
  sensors.begin();

  // Initialize BH1750 light sensor
  if (lightMeter.begin(BH1750::CONTINUOUS_HIGH_RES_MODE, 0x23, &I2C_BH1750)) {
    Serial.println(F("BH1750 initialized"));
  } else {
    Serial.println(F("Error initializing BH1750"));
  }

  // Initialize WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to the WiFi network with IP: ");
  Serial.println(WiFi.localIP());

  // Initialize web server
  server.on("/", handleRoot); // Handle root URL
  server.on("/data", handleData); // Handle data endpoint
  server.on("/update", handleUpdate); // Handle data update endpoint
  server.on("/light", handleLight); // Handle light control endpoint
  server.begin(); // Start server
  Serial.println("HTTP server started");

  // Initialize HX711
  LoadCell.begin();
  unsigned long stabilizingtime = 2000; // Precision right after power-up can be improved by adding a few seconds of stabilizing time
  boolean _tare = true; // Set this to false if you don't want tare to be performed in the next step
  LoadCell.start(stabilizingtime, _tare);
  if (LoadCell.getTareTimeoutFlag() || LoadCell.getSignalTimeoutFlag()) {
    Serial.println("Timeout, check MCU>HX711 wiring and pin designations");
    while (1);
  } else {
    LoadCell.setCalFactor(1.0); // User set calibration value (float), initial value 1.0 may be used for this sketch
    Serial.println("Startup is complete");
  }

  if (EEPROM.begin(512)) {
    EEPROM.get(calVal_eepromAdress, userWeightGrams);
    if (userWeightGrams != 0) {
      LoadCell.setCalFactor(userWeightGrams);
      Serial.print("Loaded calibration factor from EEPROM: ");
      Serial.println(userWeightGrams);
    }
  }

  calibrate(); // Start calibration procedure

  Serial.println("Enter new data through the Serial Monitor:");
}

void loop() {
  unsigned long currentMillis = millis();

  // Print sensor data to Serial Monitor every interval
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    printSensorData();
  }

  server.handleClient(); // Handle client requests

  // Update sensor data based on Serial input
  updateSensorData();

  // Read GPS data from NEO-7M module
  readGPSData();

  // Read temperature data from DS18B20 sensor
  readTemperatureData();

  // Read light level from BH1750 sensor and control the LED
  if (lightStatus) {
    readLightData();
  }

  // Read weight data from HX711 sensor
  readWeightData();
}

void handleRoot() {
  // HTTP response to root URL
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
  // HTTP response to /data endpoint
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
      digitalWrite(lightPin, LOW); // Ensure LED is off if light status is off
      Serial.println("Light turned off");
    } else {
      server.send(400, "text/plain", "Invalid state value");
      return;
    }

    server.send(200, "text/plain", "Light status updated");
  } else {
    server.send(400, "text/plain", "State parameter missing");
  }
}

void readTemperatureData() {
  sensors.requestTemperatures(); // Send command to get temperatures
  temperature = sensors.getTempCByIndex(0); // Read temperature in Celsius
}

void readLightData() {
  lux = lightMeter.readLightLevel(); // Read light level in lux
  if (lux < lightThreshold) {
    digitalWrite(lightPin, HIGH); // Turn on LED if light level is below threshold
  } else {
    digitalWrite(lightPin, LOW); // Turn off LED if light level is above threshold
  }
}

void readWeightData() {
  static boolean newDataReady = 0;
  const int serialPrintInterval = 5000; // Adjusted value for serial print activity

  // Check for new weight data
  if (LoadCell.update()) newDataReady = true;

  // Display weight data on Serial Monitor
  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      weight = LoadCell.getData();
      Serial.print("Load_cell output val: ");
      Serial.println(weight);
      newDataReady = 0;
      t = millis();
    }
  }
}

void readGPSData() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    if (gps.location.isUpdated()) {
      gpsData = String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
      Serial.println("Updated GPS data: " + gpsData);
    }
  }
}

String getRTCDateTime() {
  DateTime now = rtc.now();
  String dateTimeString = String(now.year()) + "-" + String(now.month()) + "-" + String(now.day());
  dateTimeString += " " + String(now.hour()) + ":" + String(now.minute()) + ":" + String(now.second());
  return dateTimeString;
}

void updateSensorData() {
  if (Serial.available() > 0) {
    String input = Serial.readString();
    input.trim();

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

void calibrate() {
  Serial.println("***");
  Serial.println("Start calibration:");
  Serial.println("Place a known weight on the scale and enter the weight in grams:");
  boolean calibrationFinished = false;
  while (!calibrationFinished) {
    if (Serial.available() > 0) {
      userWeightGrams = Serial.parseFloat();
      if (userWeightGrams != 0) {
        Serial.print("Known weight: ");
        Serial.print(userWeightGrams);
        Serial.println(" grams");
        LoadCell.refreshDataSet(); // Refresh the data set to be sure we're using the latest data
        float calibrationFactor = LoadCell.getNewCalibration(userWeightGrams); // Get the new calibration factor
        Serial.print("New calibration factor: ");
        Serial.println(calibrationFactor);
        LoadCell.setCalFactor(calibrationFactor); // Set the new calibration factor
        EEPROM.put(calVal_eepromAdress, calibrationFactor); // Save the new calibration factor to EEPROM
        EEPROM.commit();
        calibrationFinished = true;
      }
    }
  }
  Serial.println("Calibration finished");
  Serial.println("***");
}

void printSensorData() {
  Serial.println("Sensor Data:");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  Serial.print("Weight: ");
  Serial.println(weight);
  Serial.print("Light: ");
  Serial.print(lux);
  Serial.println(" lx");
  Serial.print("GPS: ");
  Serial.println(gpsData);
  Serial.print("Lock Status: ");
  Serial.println(lockStatus);
  Serial.print("Light Status: ");
  Serial.println(lightStatus ? "on" : "off");
  Serial.print("RTC DateTime: ");
  Serial.println(getRTCDateTime());
}
