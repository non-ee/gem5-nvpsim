#include "delay.h"
#include "peripheral.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
#include <sys/types.h>

// ==================== CONFIGURATION ====================
#define SAMPLE_COUNT 20  // 10 seconds at 10Hz sampling
#define ACCEL_DATA_DIM 3  // x, y, z axes
#define WINDOW_SIZE 5     // For moving average
#define THRESHOLD 1.5     // Step detection threshold

#define ACCELEROMETER_ID 0
#define RF_ID   1

// Accelerometer data buffer: [sample_count][x,y,z]
volatile int16_t accel_data[SAMPLE_COUNT][ACCEL_DATA_DIM];
volatile uint8_t activity_result[SAMPLE_COUNT];  // 1 if step detected, 0 otherwise
volatile uint32_t total_steps = 0;

// Simple step counter variables
volatile float magnitude_buffer[WINDOW_SIZE];
volatile int buffer_index = 0;

// Pre define
void sensing_task();
void generate_synthetic_data();
void pre_compute();
void heavy_compute();
void post_compute();
void display_output();
void post_processing();

int main() {
#ifdef W_ACCEL
    accel_map_registers();
#endif

    sensing_task();

    // For simulation, generate synthetic accelerometer data
    generate_synthetic_data();

    pre_compute();
    heavy_compute();
    post_compute();

    display_output();

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    post_processing();

    return 0;
}

void sensing_task() {
    // Simulate accelerometer sensing (10Hz sampling for 10 seconds)
    uint8_t *accel_reg;
    int16_t x, y, z;

    periRegister(ACCELEROMETER_ID, &accel_reg);
    periInit(accel_reg);

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Read accelerometer values (simplified - in real HW, these would come from sensor)
        accelSense(&x, &y, &z, accel_reg);
        accel_data[i][0] = x;
        accel_data[i][1] = y;
        accel_data[i][2] = z;
        // Simulate 3-axis accelerometer data
        // In a real system, these would be actual sensor readings
        // For simulation, we'll generate synthetic data later

        DelayMS(10);  // 10Hz sampling
    }

    periTurnOff(accel_reg);
    periLogout(ACCELEROMETER_ID);
}

void generate_synthetic_data() {
    // Generate synthetic accelerometer data for simulation
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

        accel_data[i][0] = (int16_t)((base_x + noise) * 1000);  // Convert to fixed-point
        accel_data[i][1] = (int16_t)((base_y + noise * 0.5) * 1000);
        accel_data[i][2] = (int16_t)((base_z + noise * 0.3) * 1000);
    }
}

void pre_compute() {
    // Optional: calibration or normalization
    // Remove gravity bias from z-axis (assuming stationary start)
    float z_bias = accel_data[0][2] / 1000.0;  // First sample as baseline

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Simple bias removal
        accel_data[i][2] -= (int16_t)(z_bias * 1000 * 0.8);  // Remove 80% of gravity
    }
}

void heavy_compute() {
#ifdef W_ACCEL
    // If using hardware accelerator
    accel_set_addr((uint64_t)accel_data, (uint64_t)activity_result,
                   SAMPLE_COUNT * ACCEL_DATA_DIM, SAMPLE_COUNT);
    accel_start();
#else
    // Software implementation: Step detection algorithm

    // Initialize magnitude buffer
    for (int i = 0; i < WINDOW_SIZE; i++) {
        magnitude_buffer[i] = 0.0;
    }

    for (int i = 0; i < SAMPLE_COUNT; i++) {
        // Calculate magnitude of acceleration vector
        float x = accel_data[i][0] / 1000.0;
        float y = accel_data[i][1] / 1000.0;
        float z = accel_data[i][2] / 1000.0;

        float magnitude = sqrt(x*x + y*y + z*z);

        // Update moving average buffer
        magnitude_buffer[buffer_index] = magnitude;
        buffer_index = (buffer_index + 1) % WINDOW_SIZE;

        // Calculate moving average
        float avg = 0.0;
        for (int j = 0; j < WINDOW_SIZE; j++) {
            avg += magnitude_buffer[j];
        }
        avg /= WINDOW_SIZE;

        // Simple threshold-based step detection
        if (magnitude > avg * THRESHOLD && magnitude > 1.2) {
            activity_result[i] = 1;  // Step detected

            // Debouncing: only count if previous few samples were low
            if (i > 3) {
                int prev_steps = 0;
                for (int k = 1; k <= 3; k++) {
                    if (activity_result[i - k] == 1) prev_steps++;
                }
                if (prev_steps == 0) {  // No recent steps
                    total_steps++;
                }
            } else {
                total_steps++;
            }
        } else {
            activity_result[i] = 0;  // No step
        }

        // Simulate computation intensity (optional)
        // for (int j = 0; j < 10; j++) {
        //     magnitude = sqrt(magnitude * 1.1);
        // }
    }
#endif
}

void post_compute() {
    // Aggregate results: count total steps
    // (This is already done in heavy_compute, but could do verification here)
    printf("Total steps detected: %u\n", total_steps);

    // Optional: Calculate step frequency
    if (total_steps > 1) {
        float step_freq = (total_steps * 10.0) / SAMPLE_COUNT;  // Steps per second
        printf("Step frequency: %.2f Hz\n", step_freq);
    }
}

void display_output() {
    printf("=== HAR Results ===\n");
    printf("Total steps: %u\n", total_steps);
    printf("Activity samples (first 20): ");
    for (int i = 0; i < 20 && i < SAMPLE_COUNT; i++) {
        printf("%d", activity_result[i]);
    }
    printf("...\n");
}

void post_processing() {
    uint8_t *rf_reg;

    periRegister(RF_ID, &rf_reg);
    periInit(rf_reg);

    // simple packet

    for (int i = 0; i < 4; i++) {
        rfTransmitByte(rf_reg, total_steps);
        DelayMS(10);
    }

    periLogout(RF_ID);
}
