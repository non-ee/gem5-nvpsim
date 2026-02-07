#include "delay.h"
#include "accel_reg.h"
#include "peripheral.h"
#include <stdint.h>

// ==================== MINIMAL CONFIGURATION ====================
#define BLOCK_SIZE          8       // 8x8 convolution block
#define SCALE               8
#define IMAGE_WIDTH         BLOCK_SIZE * SCALE
#define IMAGE_HEIGHT        BLOCK_SIZE * SCALE
#define IMAGE_SIZE          (IMAGE_WIDTH * IMAGE_HEIGHT)
#define SCALE_SQUARE        SCALE * SCALE
#define MOTION_THRESHOLD    40      // Scaled for 16x16

// Max size for scale=4 (32x32)
#define MAX_SIZE (8 * SCALE)
static uint8_t prev_frame[MAX_SIZE][MAX_SIZE];

#define CAMERA_ID   0
#define RF_ID       1

// ==================== GLOBAL BUFFERS ====================
volatile uint8_t input[IMAGE_HEIGHT][IMAGE_WIDTH];   // 16x16 input
volatile uint8_t output[IMAGE_HEIGHT][IMAGE_WIDTH];  // 16x16 output

void capture_image(int width, int height);
void process_image_naive(int scale);
void process_image_strategy2(int scale);
void process_image_strategy2_double_buffer(int scale);
void process_image_strategy2_hybrid(int scale);
void process_image_strategy3(int scale);
uint8_t detect_motion_simple(int scale);
void send_motion_result(uint8_t result);

// ==================== MAIN ====================
int main(void) {
#ifdef W_ACCEL
    accel_map_registers();
#endif

    // capture_image(IMAGE_WIDTH, IMAGE_HEIGHT);
    capture_image(IMAGE_WIDTH, IMAGE_HEIGHT);
    // process_image_naive(SCALE);
    // process_image_strategy2(SCALE);
    // process_image_strategy2_double_buffer(SCALE);
    process_image_strategy2_hybrid(SCALE);
    // process_image_strategy3(SCALE);
    uint8_t motion_detected = detect_motion_simple(SCALE);

    // 4. Transmitting
    send_motion_result(motion_detected);

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    return 0;
}

// ==================== SCALED SENSING ====================
void capture_image(int width, int height) {
    uint8_t *camera_reg;
    uint8_t pixel;

    // Initialize camera peripheral
    periRegister(CAMERA_ID, &camera_reg);
    periInit(camera_reg);

    // Capture 16x16 image
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            camCapturePixel(&pixel, camera_reg, x, y);
            input[y][x] = pixel;
            DelayUS(1);
        }
    }

    periTurnOff(camera_reg);
    periLogout(CAMERA_ID);
}

// ==================== 8x8 CONVOLUTION KERNEL ====================
void convolution_8x8(uint8_t in_block[8][8], uint8_t out_block[8][8]) {
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
}

// ==================== NAIVE/ORIGINAL APPROACH ====================
void process_image_naive(int scale) {
    int image_size = 8 * scale;

    for (int block_y = 0; block_y < scale; block_y++) {
        for (int block_x = 0; block_x < scale; block_x++) {
            uint8_t in_block[8][8];
            uint8_t out_block[8][8];

            // Extract 8x8 block
            int start_y = block_y * 8;
            int start_x = block_x * 8;

            for (int y = 0; y < 8; y++) {
                for (int x = 0; x < 8; x++) {
                    in_block[y][x] = input[start_y + y][start_x + x];
                }
            }

            #ifdef W_ACCEL
            accel_set_addr((uint64_t)in_block, (uint64_t)out_block, 64, 64);
            accel_start();
            #else
            convolution_8x8(in_block, out_block);
            #endif

            // Store result back
            for (int y = 0; y < 8; y++) {
                for (int x = 0; x < 8; x++) {
                    output[start_y + y][start_x + x] = out_block[y][x];
                }
            }
        }
    }
}

// ==================== HW STRATEGY : PIPELINE MULTIPLE CALLS ====================
// void process_image_hw_strategy()

// ==================== STRATEGY 2: PIPELINE MULTIPLE CALLS ====================
void process_image_strategy2(int scale) {
    int num_blocks = scale * scale;

    // Allocate arrays for all block pointers
    uint8_t* in_ptrs[num_blocks];
    uint8_t* out_ptrs[num_blocks];
    uint8_t* temp_buffers[num_blocks];

    // Pre-load all input blocks into temporary buffers
    for (int block_y = 0; block_y < scale; block_y++) {
        for (int block_x = 0; block_x < scale; block_x++) {
            int block_idx = block_y * scale + block_x;
            int start_y = block_y * 8;
            int start_x = block_x * 8;

            // Allocate temporary buffer for this block
            temp_buffers[block_idx] = malloc(8 * 8 * sizeof(uint8_t));
            in_ptrs[block_idx] = temp_buffers[block_idx];
            out_ptrs[block_idx] = (uint8_t*)&output[start_y][start_x];

            // Copy input data to temp buffer
            for (int y = 0; y < 8; y++) {
                for (int x = 0; x < 8; x++) {
                    temp_buffers[block_idx][y * 8 + x] = input[start_y + y][start_x + x];
                }
            }
        }
    }
    #ifdef W_ACCEL
    // Start all accelerator calls in rapid succession
    for (int i = 0; i < num_blocks; i++) {
        accel_set_addr((uint64_t)in_ptrs[i], (uint64_t)out_ptrs[i], 64, 64);
        accel_start();
        // No wait here - pipeline the calls if accelerator supports it
    }
    #else
    // Software fallback: Process all blocks
    for (int i = 0; i < num_blocks; i++) {
        convolution_8x8((uint8_t(*)[8])in_ptrs[i], (uint8_t(*)[8])out_ptrs[i]);
    }
    #endif

    // Clean up temporary buffers
    for (int i = 0; i < num_blocks; i++) {
        free(temp_buffers[i]);
    }
}

void process_image_strategy2_double_buffer(int scale) {
    int num_blocks = scale * scale;

    // Double buffer for output only (input is accessed directly)
    uint8_t out_buffers[2][8][8];
    int current_buffer = 0;

    for (int i = 0; i < num_blocks; i++) {
        int block_y = i / scale;
        int block_x = i % scale;
        int start_y = block_y * 8;
        int start_x = block_x * 8;

        // Direct pointer to input
        uint8_t (*in_ptr)[8] = &input[start_y];
        in_ptr = (uint8_t(*)[8])((uint8_t*)in_ptr + start_x);

        // Pointer to current output buffer
        uint8_t (*out_ptr)[8] = out_buffers[current_buffer];

        #ifdef W_ACCEL
        // Submit accelerator call
        accel_set_addr((uint64_t)in_ptr, (uint64_t)out_ptr, 64, 64);
        accel_start();

        // If not first block, wait for previous to complete and write output
        if (i > 0) {
            // Write previous output directly to memory
            int prev_idx = i - 1;
            int prev_start_y = (prev_idx / scale) * 8;
            int prev_start_x = (prev_idx % scale) * 8;

            uint8_t (*prev_out)[8] = out_buffers[(current_buffer + 1) % 2];
            for (int y = 0; y < 8; y++) {
                for (int x = 0; x < 8; x++) {
                    output[prev_start_y + y][prev_start_x + x] = prev_out[y][x];
                }
            }
        }
        #else
        // Software: Process directly
        convolution_8x8(in_ptr, out_ptr);

        // Write output directly
        for (int y = 0; y < 8; y++) {
            for (int x = 0; x < 8; x++) {
                output[start_y + y][start_x + x] = out_ptr[y][x];
            }
        }
        #endif

        // Switch buffer
        current_buffer = (current_buffer + 1) % 2;
    }

    #ifdef W_ACCEL
    // Wait for last operation and write
    int last_idx = num_blocks - 1;
    int last_start_y = (last_idx / scale) * 8;
    int last_start_x = (last_idx % scale) * 8;

    uint8_t (*last_out)[8] = out_buffers[(current_buffer + 1) % 2];
    for (int y = 0; y < 8; y++) {
        for (int x = 0; x < 8; x++) {
            output[last_start_y + y][last_start_x + x] = last_out[y][x];
        }
    }
    #endif
}

void process_image_strategy2_hybrid(int scale) {
    int num_blocks = scale * scale;

    // Use direct pointers to input/output (no temp buffers for input)
    uint8_t* in_ptrs[num_blocks];
    uint8_t* out_ptrs[num_blocks];

    // Prepare direct pointers
    for (int block_y = 0; block_y < scale; block_y++) {
        for (int block_x = 0; block_x < scale; block_x++) {
            int block_idx = block_y * scale + block_x;
            int start_y = block_y * 8;
            int start_x = block_x * 8;

            // Direct pointers to input memory
            in_ptrs[block_idx] = (uint8_t*)&input[start_y][start_x];
            out_ptrs[block_idx] = (uint8_t*)&output[start_y][start_x];
        }
    }

    #ifdef W_ACCEL
    // Submit all accelerator calls rapidly
    for (int i = 0; i < num_blocks; i++) {
        accel_set_addr((uint64_t)in_ptrs[i], (uint64_t)out_ptrs[i], 64, 64);
        accel_start();
    }
    #else
    // Software: Use direct memory access
    for (int i = 0; i < num_blocks; i++) {
        convolution_8x8((uint8_t(*)[8])in_ptrs[i], (uint8_t(*)[8])out_ptrs[i]);
    }
    #endif
}

// ==================== STRATEGY 3: DIRECT MEMORY ACCESS ====================
void process_image_strategy3(int scale) {
    // If accelerator doesn't support full image or in software mode,
    // use direct pointers without copying
    for (int block_y = 0; block_y < scale; block_y++) {
        for (int block_x = 0; block_x < scale; block_x++) {
            int start_y = block_y * 8;
            int start_x = block_x * 8;

            #ifdef W_ACCEL
            // Send directly from input to output memory locations
            accel_set_addr((uint64_t)&input[start_y][start_x],
                          (uint64_t)&output[start_y][start_x],
                          64, 64);
            accel_start();
            #else
            // Software: Process directly without copying to temp buffer
            for (int y = 1; y < 7; y++) {
                for (int x = 1; x < 7; x++) {
                    int weights[3][3] = {{1, 2, 1}, {2, 4, 2}, {1, 2, 1}};
                    int sum = 0;
                    for (int ky = -1; ky <= 1; ky++) {
                        for (int kx = -1; kx <= 1; kx++) {
                            sum += input[start_y + y + ky][start_x + x + kx] *
                                   weights[ky + 1][kx + 1];
                        }
                    }
                    output[start_y + y][start_x + x] = sum / 16;
                }
            }

            // Set border pixels
            for (int i = 0; i < 8; i++) {
                output[start_y][start_x + i] = 0;
                output[start_y + 7][start_x + i] = 0;
                output[start_y + i][start_x] = 0;
                output[start_y + i][start_x + 7] = 0;
            }
            #endif
        }
    }
}

// ==================== SCALED MOTION DETECTION ====================
// Simple threshold function that works for any scale
uint8_t detect_motion_simple(int scale) {
    int image_size = 8 * scale;
    uint16_t diff = 0;

    // Basic pixel difference accumulation
    for (int y = 0; y < image_size; y++) {
        for (int x = 0; x < image_size; x++) {
            uint8_t current = input[y][x];
            uint8_t previous = prev_frame[y][x];

            // Quick absolute difference
            diff += abs((int)current - (int)previous);

            prev_frame[y][x] = current;
        }
    }

    // Simple scaling of threshold (assuming baseline is 16x16 = 256 pixels)
    uint16_t scaled_threshold = MOTION_THRESHOLD * image_size * image_size / 256;

    return (diff > scaled_threshold) ? 1 : 0;
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
