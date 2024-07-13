#include <Arduino.h>
#include <FS.h>
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>

#define I2S_WS 15
#define I2S_SD 32
#define I2S_SCK 14
#define SD_CS 5
#define SWITCH_PIN 4

const int sampleRate = 48000;
const int bitsPerSample = 16;
const int numChannels = 1;
const int bufferSize = 512;
int fileCounter = 0;

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

void setup() {
  Serial.begin(115200);

  // Initialize the SD card
  if (!SD.begin(SD_CS)) {
    Serial.println("Card Mount Failed");
    return;
  }

  // Initialize I2S
  i2s_init();

  // Initialize switch pin
  pinMode(SWITCH_PIN, INPUT_PULLUP);
}

void loop() {
  if (digitalRead(SWITCH_PIN) == LOW) {
    // Generate a unique file name
    String fileName = "/audio" + String(fileCounter) + ".wav";
    fileCounter++;

    // Create a new WAV file
    File file = SD.open(fileName.c_str(), FILE_WRITE);
    if (!file) {
      Serial.println("Failed to create file");
      return;
    }

    // Write a placeholder header
    write_wav_header(file, 0);

    // Start recording
    Serial.println("Recording...");
    size_t bytes_written = 0;
    uint8_t buffer[bufferSize];
    while (digitalRead(SWITCH_PIN) == LOW) { // Record while the switch is on
      size_t bytes_read;
      i2s_read(I2S_NUM_0, buffer, bufferSize, &bytes_read, portMAX_DELAY);
      if (bytes_read > 0) {
        file.write(buffer, bytes_read);
        bytes_written += bytes_read;
      }
    }

    // Update the WAV header with the correct data size
    file.seek(0);
    write_wav_header(file, bytes_written);

    // Close the file
    file.close();
    Serial.println("Recording complete");
  }
}
