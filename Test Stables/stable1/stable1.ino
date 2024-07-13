
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750.h>

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

// Sensor data
float temperature = 0.0;
float weight = 70.0; // Placeholder for weight sensor data
float lux = 0.0; // Light level in lux
String gpsData = "37.7749,-122.4194"; // Initial example GPS coordinates
String lockStatus = "unlocked"; // Example lock status

unsigned long previousMillis = 0; // Variable to store the last time data was printed
const long interval = 5000; // Interval between data prints (in milliseconds)

const int lightPin = 26; // Digital pin for the LED
bool lightStatus = false; // Light status
// bool manualControl = false; // Manual control status
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

  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi..");
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to the WiFi network with IP: ");
  Serial.println(WiFi.localIP());

  server.on("/", handleRoot); // Handle root URL
  server.on("/data", handleData); // Handle data endpoint
  server.on("/update", handleUpdate); // Handle data update endpoint
  server.on("/light", handleLight); // Handle light control endpoint

  server.begin(); // Start server
  Serial.println("HTTP server started");

  Serial.println("Enter new data through the Serial Monitor:");
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
      // digitalWrite(lightPin, HIGH);
      lightStatus = true;
      // manualControl = true; // Enable manual control
      Serial.println("Light turned on");
    } else if (state == "off") {
      // digitalWrite(lightPin, LOW);
      lightStatus = false;
      // manualControl = true; // Enable manual control
      Serial.println("Light turned off");
    }
    server.send(200, "text/plain", "Light toggled");
  } else {
    server.send(400, "text/plain", "Bad Request");
  }
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
}

void readLightData() {
  lux = lightMeter.readLightLevel(); // Read light level in lux
  
  // Control the LED based on the light intensity
  if (lux < lightThreshold) {
    digitalWrite(lightPin, HIGH); // Turn on the LED if light level is below the threshold
    // lightStatus = true;
  } else {
    digitalWrite(lightPin, LOW); // Turn off the LED if light level is above the threshold
    // lightStatus = false;
  }
}


void printSensorData() {
  // Print all sensor data to Serial Monitor
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
  // Update sensor data from Serial input
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    input.trim(); // Remove any trailing newline characters

    Serial.println("Received serial input: " + input);

    // Simple parsing for demo purposes
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
  // Read GPS data from NEO-7M module and update gpsData variable
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
  sensors.requestTemperatures(); // Send command to get temperature readings
  temperature = sensors.getTempCByIndex(0); // Get temperature in Celsius
}


String getRTCDateTime() {
  // Get the current date and time from the RTC
  DateTime now = rtc.now();
  char buf[] = "YYYY-MM-DD hh:mm:ss";
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(buf);
}
