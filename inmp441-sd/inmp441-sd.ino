#include <driver/i2s.h>
#include <SD.h>
#include <SPI.h>

// User Configuration
const int RECORD_TIME_IN_SECONDS = 5;
const int SAMPLE_RATE_IN_HZ = 22050;

// Constants
const int WAV_SAMPLE_SIZE_IN_BITS = 16;
const int WAV_SAMPLE_SIZE_IN_BYTES = WAV_SAMPLE_SIZE_IN_BITS / 8;
const int MIC_SAMPLE_BUFFER_SIZE_IN_BYTES = 4096;
const int SDCARD_SAMPLE_BUFFER_SIZE_IN_BYTES = MIC_SAMPLE_BUFFER_SIZE_IN_BYTES / 2;
const int NUM_SAMPLE_BYTES_TO_WRITE = RECORD_TIME_IN_SECONDS * SAMPLE_RATE_IN_HZ * WAV_SAMPLE_SIZE_IN_BYTES;
const int NUM_SAMPLES_IN_DMA_BUFFER = 256;
const int NUM_CHANNELS = 1;

// SD Card Pin Configuration
const int SD_CS_PIN = 5;  // Adjust based on your hardware

// Buffers
uint8_t mic_samples[MIC_SAMPLE_BUFFER_SIZE_IN_BYTES];
uint8_t wav_samples[SDCARD_SAMPLE_BUFFER_SIZE_IN_BYTES];

// Variables
File wavFile;
int num_sample_bytes_written_to_wav = 0;

// Function to snip 16-bit samples from 32-bit data
int snip_16_mono(const uint8_t* samples_in, uint8_t* samples_out, int num_bytes) {
    int num_samples = num_bytes / 4;
    for (int i = 0; i < num_samples; ++i) {
        samples_out[2 * i] = samples_in[4 * i + 2];
        samples_out[2 * i + 1] = samples_in[4 * i + 3];
    }
    return num_samples * 2;
}

// Function to create WAV header
void create_wav_header(uint8_t* header, int sampleRate, int bitsPerSample, int num_channels, int num_samples) {
    int datasize = num_samples * num_channels * bitsPerSample / 8;
    header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
    uint32_t filesize = datasize + 36;
    memcpy(header + 4, &filesize, 4);
    header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
    header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
    uint32_t fmt_chunk_size = 16;
    memcpy(header + 16, &fmt_chunk_size, 4);
    uint16_t format_type = 1;
    memcpy(header + 20, &format_type, 2);
    memcpy(header + 22, &num_channels, 2);
    memcpy(header + 24, &sampleRate, 4);
    uint32_t byte_rate = sampleRate * num_channels * bitsPerSample / 8;
    memcpy(header + 28, &byte_rate, 4);
    uint16_t block_align = num_channels * bitsPerSample / 8;
    memcpy(header + 32, &block_align, 2);
    memcpy(header + 34, &bitsPerSample, 2);
    header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
    memcpy(header + 40, &datasize, 4);
}

void setup() {
    Serial.begin(115200);
    while (!Serial) {}

    // Configure I2S
    i2s_config_t i2s_config = {
        .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
        .sample_rate = SAMPLE_RATE_IN_HZ,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
        .communication_format = I2S_COMM_FORMAT_I2S_MSB,
        .intr_alloc_flags = 0, // Default interrupt priority
        .dma_buf_count = 8,
        .dma_buf_len = 64,
        .use_apll = false,
        .tx_desc_auto_clear = false,
        .fixed_mclk = 0
    };

    i2s_pin_config_t pin_config = {
        .bck_io_num = 26,   // Bit Clock (BCK) Pin
        .ws_io_num = 25,    // Word Select (WS) Pin
        .data_out_num = -1, // Not used
        .data_in_num = 34   // Serial Data In (SDIN) Pin
    };

    i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
    i2s_set_pin(I2S_NUM_0, &pin_config);
    i2s_zero_dma_buffer(I2S_NUM_0);

    // Configure SD card
    if (!SD.begin(SD_CS_PIN)) {
        Serial.println("Failed to initialize SD card!");
        while (1);
    }

    wavFile = SD.open("/mic_left_channel_16bits.wav", FILE_WRITE);
    if (!wavFile) {
        Serial.println("Failed to create WAV file!");
        while (1);
    }

    // Create and write WAV header
    uint8_t wav_header[44];
    create_wav_header(wav_header, SAMPLE_RATE_IN_HZ, WAV_SAMPLE_SIZE_IN_BITS, NUM_CHANNELS, SAMPLE_RATE_IN_HZ * RECORD_TIME_IN_SECONDS);
    wavFile.write(wav_header, 44);

    Serial.println("Starting");
}

void loop() {
    if (num_sample_bytes_written_to_wav < NUM_SAMPLE_BYTES_TO_WRITE) {
        size_t num_bytes_read_from_mic = 0;
        i2s_read(I2S_NUM_0, (void*)mic_samples, MIC_SAMPLE_BUFFER_SIZE_IN_BYTES, &num_bytes_read_from_mic, portMAX_DELAY);
        if (num_bytes_read_from_mic > 0) {
            int num_bytes_snipped = snip_16_mono(mic_samples, wav_samples, num_bytes_read_from_mic);
            int num_bytes_to_write = min(num_bytes_snipped, NUM_SAMPLE_BYTES_TO_WRITE - num_sample_bytes_written_to_wav);
            wavFile.write(wav_samples, num_bytes_to_write);
            num_sample_bytes_written_to_wav += num_bytes_to_write;
        }
    } else {
        wavFile.close();
        SD.end();
        i2s_driver_uninstall(I2S_NUM_0);
        Serial.println("Done");
        Serial.print(num_sample_bytes_written_to_wav);
        Serial.println(" sample bytes written to WAV file");
        while (1);  // Stop the loop
    }
}
