#include <driver/i2s.h>

#define I2S_WS 15
#define I2S_SCK 14
#define I2S_SD 32
#define SAMPLE_RATE 16000

void setup() {
  Serial.begin(115200);

  // I2S configuration
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Change to LEFT channel
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  // I2S pin configuration
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  // Install and start I2S driver
  esp_err_t err;

  err = i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
    Serial.printf("Failed to install I2S driver: %d\n", err);
    return;
  }

  err = i2s_set_pin(I2S_NUM_0, &pin_config);
  if (err != ESP_OK) {
    Serial.printf("Failed to set I2S pin configuration: %d\n", err);
    return;
  }

  // Optional: Set clock source for I2S
  err = i2s_set_clk(I2S_NUM_0, SAMPLE_RATE, I2S_BITS_PER_SAMPLE_16BIT, I2S_CHANNEL_MONO);
  if (err != ESP_OK) {
    Serial.printf("Failed to set I2S clock: %d\n", err);
    return;
  }

  Serial.println("I2S driver installed successfully");
}

void loop() {
  int16_t samples[64];
  size_t bytes_read;

  // Read from I2S
  esp_err_t result = i2s_read(I2S_NUM_0, &samples, sizeof(samples), &bytes_read, portMAX_DELAY);
  
  if (result == ESP_OK) {
    // Print samples to Serial Monitor
    for (int i = 0; i < bytes_read / 2; i++) {
      Serial.println(samples[i]);
    }
  } else {
    Serial.printf("Failed to read data from I2S: %d\n", result);
  }

  delay(1000); // Add a delay to avoid overwhelming the Serial Monitor
}
