#include "delay.h"
#include "peripheral.h"
#include <stdint.h>

// ==================== CONFIGURATION ====================
#define SAMPLE_COUNT       20    // Number of audio samples to collect
#define SAMPLE_INTERVAL_MS 50    // 50ms between samples = 20Hz sampling rate
#define FFT_SIZE           32    // Simple frequency analysis size

// Animal type classifications
typedef enum {
    NO_ANIMAL = 0,
    BIRD,
    MAMMAL,
    INSECT,
    UNKNOWN
} AnimalType;

// Frequency ranges for different animals (Hz)
#define BIRD_FREQ_MIN       2000
#define BIRD_FREQ_MAX       8000
#define MAMMAL_FREQ_MIN     500
#define MAMMAL_FREQ_MAX     4000
#define INSECT_FREQ_MIN     8000
#define INSECT_FREQ_MAX    15000

#define MIC_ID	0
#define RF_ID	1

// ==================== GLOBAL BUFFERS ====================
volatile uint16_t audio_samples[SAMPLE_COUNT];
volatile uint8_t frequency_bins[FFT_SIZE];
volatile AnimalType detected_animal = NO_ANIMAL;
volatile uint8_t confidence = 0;  // 0-100% confidence

// ==================== MAIN FUNCTIONS ====================
void collect_audio(void);
uint32_t analyze_frequency(void);
AnimalType classify_animal(uint32_t dominant_freq);
void transmit_result(AnimalType animal, uint8_t confidence);

// ==================== MAIN PROGRAM ====================
int main(void) {
    printf("Simple Wildlife Classifier\n");

#ifdef W_ACCEL
    printf("Using hardware acceleration\n");
    accel_map_registers();
#endif

    // 1. Collect audio samples
    collect_audio();

    // 2. Analyze frequency
    uint32_t dominant_freq = analyze_frequency();

    // 3. Classify animal
    detected_animal = classify_animal(dominant_freq);

    const char* animal_names[] = {"None", "Bird", "Mammal", "Insect", "Unknown"};
    printf("Detected: %s (Confidence: %d%%)\n",
           animal_names[detected_animal], confidence);

    // 5. Transmit result
    transmit_result(detected_animal, confidence);

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    return 0;
}

// ==================== AUDIO COLLECTION ====================
void collect_audio(void) {
    uint8_t *mic_reg;
    uint16_t sample;

    // Initialize microphone
    periRegister(MIC_ID, &mic_reg);
    periInit(mic_reg);

    // Collect samples
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        micSense(&sample, mic_reg);
        audio_samples[i] = sample;
        DelayMS(SAMPLE_INTERVAL_MS);
    }

    // Turn off microphone
    periTurnOff(mic_reg);
    periLogout(MIC_ID);
}

// ==================== FREQUENCY ANALYSIS ====================
uint32_t analyze_frequency(void) {
#ifdef W_ACCEL
    // Use hardware accelerator for FFT
    accel_set_addr((uint64_t)audio_samples, (uint64_t)frequency_bins, SAMPLE_COUNT, FFT_SIZE);
    accel_start();
    accel_wait();

    // Find strongest frequency bin
    uint8_t max_bin = 0;
    uint8_t max_value = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
        if (frequency_bins[i] > max_value) {
            max_value = frequency_bins[i];
            max_bin = i;
        }
    }

    // Convert bin to frequency (simplified)
    // Assuming sample rate = 1000/SAMPLE_INTERVAL_MS Hz
    uint32_t sample_rate = 1000 / SAMPLE_INTERVAL_MS; // 20Hz in this case
    uint32_t freq = (max_bin * sample_rate) / FFT_SIZE;

    return freq * 100; // Scale up for realistic frequencies
#else
    // Software implementation: Simple zero-crossing frequency estimation
    int zero_crossings = 0;

    // Find average to determine zero line
    uint32_t sum = 0;
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        sum += audio_samples[i];
    }
    uint16_t average = sum / SAMPLE_COUNT;

    // Count zero crossings
    int16_t prev = (int16_t)audio_samples[0] - average;
    for (int i = 1; i < SAMPLE_COUNT; i++) {
        int16_t current = (int16_t)audio_samples[i] - average;

        if ((prev < 0 && current >= 0) || (prev >= 0 && current < 0)) {
            zero_crossings++;
        }
        prev = current;
    }

    // Calculate frequency
    // Samples collected over: SAMPLE_COUNT * SAMPLE_INTERVAL_MS / 1000 seconds
    float duration_sec = (SAMPLE_COUNT * SAMPLE_INTERVAL_MS) / 1000.0f;
    uint32_t frequency = (zero_crossings / (2.0f * duration_sec));

    return frequency;
#endif
}

// ==================== ANIMAL CLASSIFICATION ====================
AnimalType classify_animal(uint32_t dominant_freq) {
    // Simple frequency-based classification
    if (dominant_freq == 0) {
        confidence = 0;
        return NO_ANIMAL;
    }

    if (dominant_freq >= BIRD_FREQ_MIN && dominant_freq <= BIRD_FREQ_MAX) {
        // Calculate confidence based on how close to center of range
        uint32_t range_center = (BIRD_FREQ_MIN + BIRD_FREQ_MAX) / 2;
        uint32_t distance = (dominant_freq > range_center) ?
                           (dominant_freq - range_center) : (range_center - dominant_freq);
        confidence = 100 - ((distance * 100) / ((BIRD_FREQ_MAX - BIRD_FREQ_MIN) / 2));
        return BIRD;
    }

    if (dominant_freq >= MAMMAL_FREQ_MIN && dominant_freq <= MAMMAL_FREQ_MAX) {
        uint32_t range_center = (MAMMAL_FREQ_MIN + MAMMAL_FREQ_MAX) / 2;
        uint32_t distance = (dominant_freq > range_center) ?
                           (dominant_freq - range_center) : (range_center - dominant_freq);
        confidence = 100 - ((distance * 100) / ((MAMMAL_FREQ_MAX - MAMMAL_FREQ_MIN) / 2));
        return MAMMAL;
    }

    if (dominant_freq >= INSECT_FREQ_MIN && dominant_freq <= INSECT_FREQ_MAX) {
        uint32_t range_center = (INSECT_FREQ_MIN + INSECT_FREQ_MAX) / 2;
        uint32_t distance = (dominant_freq > range_center) ?
                           (dominant_freq - range_center) : (range_center - dominant_freq);
        confidence = 100 - ((distance * 100) / ((INSECT_FREQ_MAX - INSECT_FREQ_MIN) / 2));
        return INSECT;
    }

    // Frequency outside known ranges
    confidence = 50;  // Low confidence for unknown
    return UNKNOWN;
}

// ==================== TRANSMISSION ====================
void transmit_result(AnimalType animal, uint8_t conf) {
    printf("Transmitting result...\n");

    uint8_t *rf_reg;
    periRegister(RF_ID, &rf_reg);
    periInit(rf_reg);

    // Simple packet: [ANIMAL_TYPE][CONFIDENCE][CHECKSUM]
    uint8_t packet[3];
    packet[0] = (uint8_t)animal;
    packet[1] = conf;
    packet[2] = packet[0] ^ packet[1];  // Simple checksum

    for (int i = 0; i < 3; i++) {
        rfTransmitByte(rf_reg, packet[i]);
        DelayMS(10);
    }

    periTurnOff(rf_reg);
    periLogout(RF_ID);
}

// ==================== HELPER FUNCTIONS ====================
// Simple FFT for software fallback (when no accelerator)
void simple_fft(uint16_t *samples, uint8_t *bins, uint32_t sample_count) {
    // Very simplified "FFT" - just bin samples by amplitude
    for (int i = 0; i < FFT_SIZE; i++) {
        bins[i] = 0;
    }

    for (int i = 0; i < sample_count; i++) {
        // Simple frequency binning based on sample pattern
        uint8_t bin = (samples[i] % FFT_SIZE);
        bins[bin]++;

        // Also look at rate of change as frequency indicator
        if (i > 0) {
            int16_t diff = samples[i] - samples[i-1];
            uint8_t diff_bin = (abs(diff) % FFT_SIZE);
            bins[diff_bin] += 2;  // Weight changes more heavily
        }
    }
}
