#include "delay.h"
#include "peripheral.h"
#include <stdint.h>

// ==================== MINIMAL CONFIGURATION ====================
#define IMAGE_WIDTH         8
#define IMAGE_HEIGHT        8
#define IMAGE_SIZE          (IMAGE_WIDTH * IMAGE_HEIGHT)
#define ITERATIONS          16  // Run kernel many times
#define KERNEL_SIZE         3       // 3x3 convolution
#define MOTION_THRESHOLD    10

#define CAMERA_ID   0
#define RF_ID       1

// ==================== GLOBAL BUFFERS ====================
volatile uint8_t input[IMAGE_HEIGHT][IMAGE_WIDTH];   // 2D array for camera capture
volatile uint8_t output[IMAGE_HEIGHT][IMAGE_WIDTH];  // 2D array for output

// ==================== KERNEL FUNCTION ====================
void convolution_kernel(void) {
#ifdef W_ACCEL
    // Hardware accelerator: Single call
    // Cast 2D arrays to uint64_t pointers
    accel_set_addr((uint64_t)input, (uint64_t)output, IMAGE_SIZE, IMAGE_SIZE);
    accel_start();
    // Optionally: accel_wait(); // If you need to wait for completion
#else
    // Software implementation: Simple 3x3 convolution
    // This is the core computation you'll implement in gem5

    // Process inner 6x6 region of 8x8 image
    for (int y = 1; y < IMAGE_HEIGHT - 1; y++) {
        for (int x = 1; x < IMAGE_WIDTH - 1; x++) {
            // 3x3 convolution with fixed weights (Gaussian-like)
            // This is the computational pattern for gem5 accelerator

            // Fixed kernel weights (simplified Gaussian)
            int weights[3][3] = {
                {1, 2, 1},
                {2, 4, 2},
                {1, 2, 1}
            };

            // Convolution: multiply-accumulate operations
            int sum = 0;

            // 3x3 convolution window
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    sum += input[y + ky][x + kx] * weights[ky + 1][kx + 1];
                }
            }

            // Normalize and store result
            output[y][x] = sum / 16;
        }
    }
#endif
}

// ==================== SIMPLIFIED SENSING ====================
void capture_image(void) {
    uint8_t *camera_reg;
    uint8_t pixel;

    // Initialize camera peripheral
    periRegister(CAMERA_ID, &camera_reg);
    periInit(camera_reg);

    // Capture image row by row using camCapturePixel
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            camCapturePixel(&pixel, camera_reg, x, y);
            input[y][x] = pixel;
            DelayUS(1);
        }
    }

    periTurnOff(camera_reg);
    periLogout(CAMERA_ID);
}

// ==================== DETECT MOTION ====================
// Between capture_image() and convolution_kernel()
uint8_t detect_motion(void) {
    static uint8_t prev_frame[8][8];
    uint16_t diff = 0;

    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            diff += (input[y][x] > prev_frame[y][x]) ?
                   (input[y][x] - prev_frame[y][x]) :
                   (prev_frame[y][x] - input[y][x]);
            prev_frame[y][x] = input[y][x];
        }
    }

    return (diff > MOTION_THRESHOLD) ? 1 : 0;
}

// ==================== DATA TRANSMISSION ====================
void send_result(uint8_t size) {
    uint8_t *rf_reg;
    periRegister(RF_ID, &rf_reg);
    periInit(rf_reg);

    // Send all bytes
    for (int i = 0; i < size; i++) {
        rfTransmitByte(rf_reg);
        DelayUS(10);
    }

    periTurnOff(rf_reg);
    periLogout(RF_ID);
}

// ==================== MAIN ====================
int main(void) {
#ifdef W_ACCEL
    accel_map_registers();
#endif

    // 1. Sensing (minimal)
    capture_image();

    // 2. Computing (repeated kernel execution - this is what you'll measure)
    for (int i = 0; i < ITERATIONS; i++) {
        convolution_kernel();
    }


    uint8_t detection_result = detect_motion();

    // 3. Transmitting (minimal)
    send_result(2);

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    return 0;
}
