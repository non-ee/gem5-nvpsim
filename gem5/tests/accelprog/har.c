#include "delay.h"
#include "peripheral.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>

// ==================== CONFIGURATION ====================
#define SAMPLE_COUNT 20          // 2 seconds at 10Hz sampling (since SAMPLE_COUNT=20)
#define ACCEL_DATA_DIM 3         // x, y, z axes
#define MOVING_AVG_WINDOW 5      // For moving average filter
#define STEP_THRESHOLD 1.5       // Step detection threshold multiplier
#define MIN_STEP_MAGNITUDE 1.2   // Minimum acceleration magnitude for step

#define ACCELEROMETER_ID 0
#define RF_ID   1

// Accelerometer data buffer: [sample_count][x,y,z]
volatile int16_t accel_data[SAMPLE_COUNT][ACCEL_DATA_DIM];
volatile uint8_t step_detected[SAMPLE_COUNT];  // 1 if step detected, 0 otherwise
volatile uint32_t step_count = 0;

// Moving average filter variables
volatile float magnitude_history[MOVING_AVG_WINDOW];
volatile int history_index = 0;

// Function prototypes
void har_sensing_phase();
void har_generate_walking_pattern();
void har_remove_gravity_bias();
void har_detect_steps();
void har_analyze_results();
void har_display_statistics();
void har_transmit_results();

int main() {
#ifdef W_ACCEL
    accel_map_registers();
#endif

    printf("=== HAR System: Step Counting ===\n");

    // Phase 1: Data Acquisition
    har_sensing_phase();

    // Phase 2: Data Preparation
    har_generate_walking_pattern();  // For simulation only
    har_remove_gravity_bias();

    // Phase 3: Step Detection
    har_detect_steps();

    // Phase 4: Result Analysis
    har_analyze_results();
    har_display_statistics();

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    // Phase 5: Communication
    har_transmit_results();

    printf("=== HAR System Complete ===\n");
    return 0;
}

// Phase 1: Data Acquisition
void har_sensing_phase() {
    printf("[HAR] Starting accelerometer sensing...\n");
    uint8_t *accel_reg;
    int16_t x, y, z;

    periRegister(ACCELEROMETER_ID, &accel_reg);
    periInit(accel_reg);

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Read accelerometer values
        accelSense(&x, &y, &z, accel_reg);
        accel_data[i][0] = x;  // X-axis
        accel_data[i][1] = y;  // Y-axis
        accel_data[i][2] = z;  // Z-axis (vertical)

        DelayMS(10);  // 10Hz sampling rate
    }

    periTurnOff(accel_reg);
    periLogout(ACCELEROMETER_ID);
    printf("[HAR] Sensing complete: %d samples acquired\n", SAMPLE_COUNT);
}

// Phase 2a: Generate synthetic walking data (for simulation only)
void har_generate_walking_pattern() {
    printf("[HAR] Generating walking pattern simulation...\n");
    // Simulates walking pattern: peaks every ~20 samples (~2 seconds between steps)
    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Base values with some noise
        float base_x = 0.0;
        float base_y = 0.0;
        float base_z = 9.8;  // Gravity

        // Add walking pattern (steps)
        if (i % 20 == 0) {  // Step every ~2 seconds
            base_x += 3.0;  // Forward acceleration
            base_y += 1.0;  // Sideways motion
            base_z += 2.0;  // Vertical bounce
        }

        // Add some noise
        float noise = ((i % 10) - 5) * 0.2;

        accel_data[i][0] = (int16_t)((base_x + noise) * 1000);
        accel_data[i][1] = (int16_t)((base_y + noise * 0.5) * 1000);
        accel_data[i][2] = (int16_t)((base_z + noise * 0.3) * 1000);
    }
}

// Phase 2b: Remove gravity bias
void har_remove_gravity_bias() {
    printf("[HAR] Removing gravity bias...\n");
    // Remove gravity bias from z-axis (assuming stationary start)
    float z_bias = accel_data[0][2] / 1000.0;  // First sample as baseline

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Simple bias removal
        accel_data[i][2] -= (int16_t)(z_bias * 1000 * 0.8);  // Remove 80% of gravity
    }
}

// Phase 3: Step detection algorithm
void har_detect_steps() {
    printf("[HAR] Detecting steps...\n");

#ifdef W_ACCEL
    printf("[HAR] computing detection with acceleration...\n");
    // Hardware accelerator path
    accel_set_addr((uint64_t)accel_data, (uint64_t)step_detected,
                   SAMPLE_COUNT * ACCEL_DATA_DIM, SAMPLE_COUNT);
    accel_start();
#else
    // Software implementation: Step detection algorithm
    printf("[HAR] computing detection with CPU...\n");

    // Initialize magnitude history buffer
    for (int i = 0; i < MOVING_AVG_WINDOW; i++) {
        magnitude_history[i] = 0.0;
    }

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Calculate magnitude of acceleration vector
        float x = accel_data[i][0] / 1000.0;
        float y = accel_data[i][1] / 1000.0;
        float z = accel_data[i][2] / 1000.0;

        float acceleration_magnitude = sqrt(x*x + y*y + z*z);

        // Update moving average filter
        magnitude_history[history_index] = acceleration_magnitude;
        history_index = (history_index + 1) % MOVING_AVG_WINDOW;

        // Calculate moving average
        float moving_average = 0.0;
        for (int j = 0; j < MOVING_AVG_WINDOW; j++) {
            moving_average += magnitude_history[j];
        }
        moving_average /= MOVING_AVG_WINDOW;

        // Threshold-based step detection
        if (acceleration_magnitude > moving_average * STEP_THRESHOLD &&
            acceleration_magnitude > MIN_STEP_MAGNITUDE) {
            step_detected[i] = 1;  // Step detected

            // Debouncing: only count if previous few samples were low
            if (i > 3) {
                int recent_steps = 0;
                for (int k = 1; k <= 3; k++) {
                    if (step_detected[i - k] == 1) recent_steps++;
                }
                if (recent_steps == 0) {  // No recent steps
                    step_count++;
                }
            } else {
                step_count++;
            }
        } else {
            step_detected[i] = 0;  // No step
        }

        // Optional: Simulate computation intensity
        // for (int j = 0; j < 10; j++) {
        //     acceleration_magnitude = sqrt(acceleration_magnitude * 1.1);
        // }
    }
#endif
    printf("[HAR] Step detection complete\n");
}

// Phase 4a: Analyze step detection results
void har_analyze_results() {
    printf("[HAR] Analyzing results...\n");

    // Calculate step frequency
    if (step_count > 1) {
        float step_frequency = (step_count * 10.0) / SAMPLE_COUNT;  // Steps per second
        float steps_per_minute = step_frequency * 60.0;
        printf("Step frequency: %.2f Hz (%.1f steps/min)\n",
               step_frequency, steps_per_minute);
    }
}

// Phase 4b: Display statistics
void har_display_statistics() {
    printf("\n=== HAR Statistics ===\n");
    printf("Total steps detected: %u\n", step_count);
    printf("Step detection pattern: ");

    int steps_in_display = (SAMPLE_COUNT < 20) ? SAMPLE_COUNT : 20;
    for (int i = 0; i < steps_in_display; i++) {
        printf("%c", step_detected[i] ? 'S' : '.');
    }

    if (SAMPLE_COUNT > 20) {
        printf("...");
    }
    printf("\n");

    // Show sample information
    printf("Samples analyzed: %d\n", SAMPLE_COUNT);
    printf("Sampling rate: 10 Hz\n");
    printf("Duration: %.1f seconds\n", SAMPLE_COUNT / 10.0);
}

// Phase 5: Transmit results via RF
void har_transmit_results() {
    printf("[HAR] Transmitting results...\n");
    uint8_t *rf_reg;

    periRegister(RF_ID, &rf_reg);
    periInit(rf_reg);

    // Transmit step count (4 bytes)
    printf("Transmitting step count: %u\n", step_count);
    for (int i = 0; i < 4; i++) {
        // Send each byte of the step count
        uint8_t byte_to_send = (step_count >> (i * 8)) & 0xFF;
        rfTransmitByte(rf_reg, byte_to_send);
        DelayMS(10);
    }

    periTurnOff(rf_reg);
    periLogout(RF_ID);
    printf("[HAR] Transmission complete\n");
}
