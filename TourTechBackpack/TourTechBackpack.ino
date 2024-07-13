
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <BH1750.h>
#include <HX711_ADC.h>
#if defined(ESP32)
#include <EEPROM.h>
#endif
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>

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
const int buzzerPin = 27; // Buzzer signal pin foR HX7711 and ReedSensor


// HX711 constructor:
HX711_ADC LoadCell(HX711_dout, HX711_sck);

const int calVal_eepromAdress = 0;
unsigned long t = 0;
float userWeightGrams = 50000; // Variable to store user weight

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
// bool manualControl = false; // Manual control status
const float lightThreshold = 10.0; // Light intensity threshold for LED control

// Two separate I2C buses
TwoWire I2C_RTC = TwoWire(0);
TwoWire I2C_BH1750 = TwoWire(1);


//pins for reed sensor and siren
const int reedSwitchPin = 4;

enum State { OPEN, CLOSED };
State currentState;

// I2S and SD definitions
#define I2S_WS 25
#define I2S_SD 33
#define I2S_SCK 32
#define SD_CS 5
#define SWITCH_PIN 34

const int sampleRate = 48000;
const int bitsPerSample = 16;
const int numChannels = 1;
const int bufferSize = 512;
int fileCounter = 0;

// Global variables for recording state
bool isRecording = false;
File recordingFile;
unsigned long recordingStartTime = 0;
const unsigned long MAX_RECORDING_DURATION = 1500000; // 5 minutes max

// WAV file header structure
struct wav_header {
  char riff_header[4];    // "RIFF"
  int wav_size;           // Size of the entire file
  char wave_header[4];    // "WAVE"
  char fmt_header[4];     // "fmt "
  int fmt_chunk_size;     // Size of the fmt chunk
  short audio_format;     // Audio format (1 for PCM)
  short num_channels;     // Number of channels
  int sample_rate;        // Sampling rate
  int byte_rate;          // Byte rate
  short sample_alignment; // Block align
  short bit_depth;        // Bits per sample
  char data_header[4];    // "data"
  int data_bytes;         // Size of the data section
};

// Initialize the I2S driver
void i2s_init() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = sampleRate,
    .bits_per_sample = (i2s_bits_per_sample_t)bitsPerSample,
    .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
    .communication_format = (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = bufferSize,
    .use_apll = false,
    .tx_desc_auto_clear = true
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
}


void setup() {
  Serial.begin(115200);
  ss.begin(9600, SERIAL_8N1, 16, 17); // NEO-7M RX, TX
  pinMode(lightPin, OUTPUT); // Set BH1750 LED pin as output
  digitalWrite(lightPin, LOW); // Ensure BH1750 LED is off initially

  pinMode(buzzerPin, OUTPUT);  // Set HX711 Buzzer pin as output
  digitalWrite(buzzerPin, LOW); // Turn off HX711 buzzer initially

  pinMode(SWITCH_PIN, INPUT_PULLUP);// Initialize switch pin


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
  
  LoadCell.begin();
  float calibrationValue;
#if defined(ESP32)
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

  
  Serial.println("System ready. Place items on the scale.");


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

  // Initialize I2S and SD
  i2s_init();
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }
  uint8_t cardType = SD.cardType();
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }
  Serial.println("SD card initialized");

  // Set the reed switch pin as an input with an internal pull-up resistor
  pinMode(reedSwitchPin, INPUT_PULLUP);

  // Initialize the state
  currentState = OPEN;

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

    Serial.println("Received update:");
    Serial.println(body);

    // Try to parse the body as a float
    float newWeight = body.toFloat();
    if (newWeight > 0) {
      userWeightGrams = newWeight;
      Serial.print("User weight set to: ");
      Serial.print(userWeightGrams);
      Serial.println(" grams");
    } else {
      Serial.println("Invalid weight. Please enter a positive number.");
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
      readLightData();
      // manualControl = true; // Enable manual control
      Serial.println("Light turned on");
    } else if (state == "off") {
      digitalWrite(lightPin, LOW);
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


  // Handle recording
  if (digitalRead(SWITCH_PIN) == LOW && !isRecording) {
    startRecording();
  } else if (digitalRead(SWITCH_PIN) == HIGH && isRecording) {
    stopRecording();
  }

  if (isRecording) {
    continueRecording();
  }


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

  //Read weight sensir data after taking user weight
  readWeightSensor();

  readReedSensor();
  
}

void readReedSensor(){
  // Read the state of the reed switch
  int reedState = digitalRead(reedSwitchPin);

  // Update the state based on the reed switch status
  if (reedState == LOW && currentState != CLOSED) {
    currentState = CLOSED;
    lockStatus = "Locked";
    // Serial.println("State changed to: CLOSED");
  } else if (reedState == HIGH && currentState != OPEN) {
    currentState = OPEN;
    lockStatus = "unlocked";
    // Serial.println("State changed to: OPEN");
  }

  // Check WiFi connection status and reed switch state to control the LED
  if (WiFi.status() != WL_CONNECTED && currentState == OPEN) {
    tone(buzzerPin, 1000, 500);  // Turn Buzzer on
  } else {
    noTone(buzzerPin); // Turn Buzzer off
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

void readWeightSensor(){
  static boolean newDataReady = 0;
  const int serialPrintInterval = 0;

  if (LoadCell.update()) newDataReady = true;

  if (newDataReady) {
    if (millis() > t + serialPrintInterval) {
      weight = LoadCell.getData();
      // Serial.print("Load_cell output val: ");
      // Serial.println(weight);

      // Check weight condition
      if (weight > userWeightGrams * 0.20) {
        tone(buzzerPin, 1000);  // Turn on buzzer at 1kHz
        // Serial.println("Weight exceeds user weight. Buzzer turned on.");
      } else {
        noTone(buzzerPin);  // Turn off buzzer
        // Serial.println("Weight is within limit. Buzzer turned off.");
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


// Write WAV header to file
void write_wav_header(File file, int data_bytes) {
  wav_header header;
  memcpy(header.riff_header, "RIFF", 4);
  header.wav_size = data_bytes + sizeof(wav_header) - 8;
  memcpy(header.wave_header, "WAVE", 4);
  memcpy(header.fmt_header, "fmt ", 4);
  header.fmt_chunk_size = 16;
  header.audio_format = 1;
  header.num_channels = numChannels;
  header.sample_rate = sampleRate;
  header.byte_rate = sampleRate * numChannels * bitsPerSample / 8;
  header.sample_alignment = numChannels * bitsPerSample / 8;
  header.bit_depth = bitsPerSample;
  memcpy(header.data_header, "data", 4);
  header.data_bytes = data_bytes;

  file.write((const uint8_t *)&header, sizeof(wav_header));
}

void startRecording() {
  if (isRecording) return;

  String fileName = "/" + getFileNameFromRTC() + ".wav";
  recordingFile = SD.open(fileName.c_str(), FILE_WRITE);
  if (!recordingFile) {
    Serial.println("Failed to create file");
    return;
  }

  write_wav_header(recordingFile, 0);
  isRecording = true;
  recordingStartTime = millis();
  Serial.println("Recording started...");
}

void continueRecording() {
  if (!isRecording) return;

  uint8_t buffer[bufferSize];
  size_t bytes_read;
  i2s_read(I2S_NUM_0, buffer, bufferSize, &bytes_read, 0);
  if (bytes_read > 0) {
    recordingFile.write(buffer, bytes_read);
  }

  if (millis() - recordingStartTime > MAX_RECORDING_DURATION) {
    stopRecording();
  }
}

void stopRecording() {
  if (!isRecording) return;

  size_t fileSize = recordingFile.size();
  recordingFile.seek(0);
  write_wav_header(recordingFile, fileSize - sizeof(wav_header));

  recordingFile.close();
  isRecording = false;
  Serial.println("Recording complete");
}

String getFileNameFromRTC() {
  DateTime now = rtc.now();
  char buf[20];
  snprintf(buf, sizeof(buf), "%04d%02d%02d_%02d%02d%02d", 
           now.year(), now.month(), now.day(), 
           now.hour(), now.minute(), now.second());
  return String(buf);
}

String getRTCDateTime() {
  // Get the current date and time from the RTC
  DateTime now = rtc.now();
  char buf[] = "YYYY-MM-DD hh:mm:ss";
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(buf);
}
