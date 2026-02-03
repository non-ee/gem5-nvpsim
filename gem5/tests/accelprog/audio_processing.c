#include "delay.h"
#include "peripheral.h"
#include <stdint.h>
#include <math.h>

// ==================== MINIMAL CONFIGURATION ====================
#define SAMPLE_COUNT       32      // 64 audio samples (similar to 8x8 image)
#define FFT_SIZE           32      // FFT output size
#define ITERATIONS         10      // Run kernel many times
#define FREQ_BINS          8       // Number of frequency bands for transmission

#define MIC_ID     0
#define RF_ID      1

// ==================== GLOBAL BUFFERS ====================
volatile int16_t audio_samples[SAMPLE_COUNT];      // Raw audio samples
volatile uint16_t fft_output[FFT_SIZE];            // FFT magnitude spectrum
volatile uint8_t freq_features[FREQ_BINS];         // Compressed frequency features

// ==================== KERNEL FUNCTION ====================
void audio_processing_kernel(void) {
#ifdef W_ACCEL
    // Hardware accelerator: Single call for FFT + feature extraction
    accel_set_addr((uint64_t)audio_samples, (uint64_t)freq_features,
                   SAMPLE_COUNT * sizeof(int16_t), FREQ_BINS);
    accel_start();
    // Optionally: accel_wait(); // If you need to wait for completion
#else
    // Software implementation: Simple FFT approximation
    // This is the core computation you'll implement in gem5

    // Step 1: Simple FFT approximation (Goertzel algorithm for key frequencies)
    // Process 8 frequency bands (0-7) representing different audio ranges
    for (int band = 0; band < FREQ_BINS; band++) {
        // Target frequency for this band (scaled for 64 samples)
        float target_freq = band * 4.0f;  // 0, 4, 8, 12, 16, 20, 24, 28 "frequency units"

        // Goertzel algorithm (simplified for fixed-point)
        float coeff = 2.0f * cos(2.0f * 3.14159f * target_freq / SAMPLE_COUNT);
        float q0 = 0, q1 = 0, q2 = 0;

        for (int n = 0; n < SAMPLE_COUNT; n++) {
            // Convert sample to float in range [-1, 1]
            float sample = audio_samples[n] / 32768.0f;

            q0 = coeff * q1 - q2 + sample;
            q2 = q1;
            q1 = q0;
        }

        // Calculate magnitude
        float magnitude = sqrt(q1*q1 + q2*q2 - q1*q2*coeff);

        // Convert to 8-bit feature value (0-255)
        freq_features[band] = (uint8_t)(magnitude * 1000);
        if (freq_features[band] > 255) freq_features[band] = 255;
    }

    // Step 2: Simple volume calculation (RMS)
    int32_t sum_sq = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        int16_t sample = audio_samples[i];
        sum_sq += sample * sample;
    }

    // Store volume in first feature slot (overwrites band 0)
    uint16_t rms = sqrt(sum_sq / SAMPLE_COUNT);
    freq_features[0] = (rms > 255) ? 255 : (uint8_t)rms;
#endif
}

// ==================== SIMPLIFIED SENSING ====================
void capture_audio(void) {
    uint8_t *mic_reg;
    int16_t sample;

    // Initialize microphone peripheral
    periRegister(MIC_ID, &mic_reg);
    periInit(mic_reg);

    micSense(&sample, mic_reg);

    periTurnOff(mic_reg);
    periLogout(MIC_ID);
}

// ==================== DATA COMPRESSION ====================
uint8_t compress_audio_features(void) {
    uint8_t compressed[16];           // Compression buffer (8 features → 16 bytes max)
    uint8_t pos = 0;                  // Position in compressed buffer

    // Simple compression: Encode each feature with its index
    for (int i = 0; i < FREQ_BINS; i++) {
        // Only transmit significant features (above threshold)
        if (freq_features[i] > 10) {
            compressed[pos++] = i;               // Frequency band index (0-7)
            compressed[pos++] = freq_features[i]; // Magnitude value (0-255)
        }
    }

    // Add terminator byte
    compressed[pos++] = 0xFF;

    return pos;  // Compressed size
}

// ==================== DATA TRANSMISSION ====================
void send_audio_data(uint8_t size) {
    uint8_t *rf_reg;
    periRegister(RF_ID, &rf_reg);
    periInit(rf_reg);

    // Send compressed audio features byte by byte
    for (int i = 0; i < size; i++) {
        rfTransmitByte(rf_reg);
        DelayUS(10);  // Small delay between bytes
    }

    periTurnOff(rf_reg);
    periLogout(RF_ID);
}

// ==================== MAIN ====================
int main(void) {
#ifdef W_ACCEL
    accel_map_registers();
#endif

    // 1. Sensing: Capture audio samples
    capture_audio();

    // 2. Computing: Process audio (repeated kernel execution)
    for (int i = 0; i < ITERATIONS; i++) {
        audio_processing_kernel();
    }

    // 3. Compression: Prepare features for transmission
    uint8_t compressed_size = compress_audio_features();

    // 4. Transmission: Send processed audio data
    send_audio_data(compressed_size);

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    return 0;
}
