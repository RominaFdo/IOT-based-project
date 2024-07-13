#include <driver/i2s.h>
#include <SD.h>
#include <SPI.h>

#define I2S_PORT           I2S_NUM_0
#define I2S_BCK_PIN        33
#define I2S_WS_PIN         25
#define I2S_DATA_IN_PIN    32
#define SD_CS_PIN          5
#define SWITCH_PIN         2

#define SAMPLE_RATE        44100
#define BITS_PER_SAMPLE    16
#define CHANNELS           1

File audioFile;
bool isRecording = false;

void writeWavHeader(File file, uint32_t sampleRate, uint16_t bitsPerSample, uint16_t channels) {
  // ... (Keep the existing writeWavHeader function implementation)
}

void updateWavHeader(File file) {
  // ... (Keep the existing updateWavHeader function implementation)
}

void setup() {
  Serial.begin(115200);
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card initialization failed!");
    return;
  }
  Serial.println("SD card initialized.");

  // Configure the I2S interface
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCK_PIN,
    .ws_io_num = I2S_WS_PIN,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DATA_IN_PIN
  };

  i2s_set_pin(I2S_PORT, &pin_config);
}

void loop() {
  if (digitalRead(SWITCH_PIN) == LOW) {
    if (!isRecording) {
      startRecording();
    } else {
      stopRecording();
    }
    delay(500); // Debounce delay
  }

  if (Serial.available() > 0) {
    char command = Serial.read();
    if (command == 'd') {
      downloadFile("/audio.wav");
    }
  }
}

void startRecording() {
  isRecording = true;
  Serial.println("Recording started.");
  
  // Open a new file for writing
  audioFile = SD.open("/audio.wav", FILE_WRITE);
  if (!audioFile) {
    Serial.println("Failed to open file for writing.");
    return;
  }

  // Write WAV header
  writeWavHeader(audioFile, SAMPLE_RATE, BITS_PER_SAMPLE, CHANNELS);

  // Start recording
  int16_t samples[1024];
  size_t bytes_read;
  while (isRecording) {
    i2s_read(I2S_PORT, &samples, sizeof(samples), &bytes_read, portMAX_DELAY);
    audioFile.write((uint8_t*)samples, bytes_read);
  }
}

void stopRecording() {
  isRecording = false;
  Serial.println("Recording stopped.");
  
  updateWavHeader(audioFile);
  audioFile.close();
  Serial.println("File closed.");
  Serial.println("Send 'd' over serial to download the file.");
}

void downloadFile(const char* fileName) {
  File file = SD.open(fileName);
  if (!file) {
    Serial.println("Failed to open file for reading.");
    return;
  }

  Serial.println("Downloading file...");
  while (file.available()) {
    Serial.write(file.read());
  }
  file.close();
  Serial.println("Download completed.");
}