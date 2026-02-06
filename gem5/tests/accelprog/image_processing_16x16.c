#include "delay.h"
#include "accel_reg.h"
#include "peripheral.h"
#include <stdint.h>

// ==================== MINIMAL CONFIGURATION ====================
#define BLOCK_SIZE          8       // 8x8 convolution block
#define IMAGE_WIDTH         16
#define IMAGE_HEIGHT        16
#define IMAGE_SIZE          (IMAGE_WIDTH * IMAGE_HEIGHT)
#define NUM_BLOCKS_X        (IMAGE_WIDTH / BLOCK_SIZE)   // 2 blocks
#define NUM_BLOCKS_Y        (IMAGE_HEIGHT / BLOCK_SIZE)  // 2 blocks
#define TOTAL_BLOCKS        (NUM_BLOCKS_X * NUM_BLOCKS_Y) // 4 blocks
#define MOTION_THRESHOLD    40      // Scaled for 16x16

#define CAMERA_ID   0
#define RF_ID       1

// ==================== GLOBAL BUFFERS ====================
volatile uint8_t input[IMAGE_HEIGHT][IMAGE_WIDTH];   // 16x16 input
volatile uint8_t output[IMAGE_HEIGHT][IMAGE_WIDTH];  // 16x16 output

// ==================== 8x8 CONVOLUTION KERNEL ====================
void convolution_8x8(uint8_t in_block[8][8], uint8_t out_block[8][8]) {
#ifdef W_ACCEL
    // Hardware accelerator for 8x8 block
    accel_set_addr((uint64_t)in_block, (uint64_t)out_block, 64, 64);
    accel_start();
#else
    // Software implementation: Simple 3x3 convolution on 8x8 block
    for (int y = 1; y < 7; y++) {
        for (int x = 1; x < 7; x++) {
            int weights[3][3] = {
                {1, 2, 1},
                {2, 4, 2},
                {1, 2, 1}
            };

            int sum = 0;
            for (int ky = -1; ky <= 1; ky++) {
                for (int kx = -1; kx <= 1; kx++) {
                    sum += in_block[y + ky][x + kx] * weights[ky + 1][kx + 1];
                }
            }
            out_block[y][x] = sum / 16;
        }
    }

    // Set border pixels
    for (int i = 0; i < 8; i++) {
        out_block[0][i] = 0;
        out_block[7][i] = 0;
        out_block[i][0] = 0;
        out_block[i][7] = 0;
    }
#endif
}

// ==================== 16x16 PROCESSING USING 8x8 BLOCKS ====================
void process_16x16_image(void) {
    // Process 16x16 image as 4 blocks of 8x8 (2x2 grid)

    for (int block_y = 0; block_y < NUM_BLOCKS_Y; block_y++) {
        for (int block_x = 0; block_x < NUM_BLOCKS_X; block_x++) {
            uint8_t in_block[8][8];
            uint8_t out_block[8][8];

            // Extract 8x8 block from 16x16 input
            int start_y = block_y * BLOCK_SIZE;
            int start_x = block_x * BLOCK_SIZE;

            for (int y = 0; y < 8; y++) {
                for (int x = 0; x < 8; x++) {
                    in_block[y][x] = input[start_y + y][start_x + x];
                }
            }

            // Process the 8x8 block
            convolution_8x8(in_block, out_block);

            // Store result back to 16x16 output
            for (int y = 0; y < 8; y++) {
                for (int x = 0; x < 8; x++) {
                    output[start_y + y][start_x + x] = out_block[y][x];
                }
            }
        }
    }
}

// ==================== SCALED SENSING ====================
void capture_16x16_image(void) {
    uint8_t *camera_reg;
    uint8_t pixel;

    // Initialize camera peripheral
    periRegister(CAMERA_ID, &camera_reg);
    periInit(camera_reg);

    // Capture 16x16 image
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

// ==================== SCALED MOTION DETECTION ====================
uint8_t detect_motion_16x16(void) {
    static uint8_t prev_frame[16][16];
    uint16_t diff = 0;

    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            uint8_t current = input[y][x];
            uint8_t previous = prev_frame[y][x];

            if (current > previous) {
                diff += current - previous;
            } else {
                diff += previous - current;
            }

            prev_frame[y][x] = current;
        }
    }

    return (diff > MOTION_THRESHOLD) ? 1 : 0;
}

// ==================== DATA TRANSMISSION ====================
void send_motion_result(uint8_t motion_detected) {
    uint8_t *rf_reg;
    periRegister(RF_ID, &rf_reg);
    periInit(rf_reg);

    // Send motion result (1 byte)
    rfTransmitByte(rf_reg);
    DelayUS(10);

    periTurnOff(rf_reg);
    periLogout(RF_ID);
}

// ==================== MAIN ====================
int main(void) {
#ifdef W_ACCEL
    accel_map_registers();
#endif

    capture_16x16_image();
    process_16x16_image();
    uint8_t motion_detected = detect_motion_16x16();

    // 4. Transmitting
    send_motion_result(motion_detected);

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    return 0;
}
