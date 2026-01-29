#include "delay.h"
#include "peripheral.h"
#include <stdint.h>

// ==================== CONFIGURATION ====================
#define IMAGE_WIDTH         8    // Image width in pixels
#define IMAGE_HEIGHT        8    // Image height in pixels
#define IMAGE_SIZE          (IMAGE_WIDTH * IMAGE_HEIGHT)
#define KERNEL_SIZE         3      // Convolution kernel size (3x3)
#define EDGE_THRESHOLD      50     // Edge detection threshold
#define NOISE_THRESHOLD     20     // Noise filtering threshold

// Filter operation modes
typedef enum {
    FILTER_GAUSSIAN_BLUR = 0,
    FILTER_EDGE_DETECT,
    FILTER_SHARPEN,
    FILTER_MEDIAN,
    FILTER_NONE
} FilterType;

// Detection results
typedef enum {
    NO_FEATURE = 0,
    EDGES_DETECTED,
    CORNERS_DETECTED,
    PATTERN_FOUND,
    MOTION_DETECTED
} FeatureType;

#define CAMERA_ID       0
#define DISPLAY_ID      1
#define RF_ID           1

// Gaussian kernel (3x3)
const int8_t gaussian_kernel[KERNEL_SIZE][KERNEL_SIZE] = {
    {1, 2, 1},
    {2, 4, 2},
    {1, 2, 1}
};

// Sobel edge detection kernels (3x3)
const int8_t sobel_x[KERNEL_SIZE][KERNEL_SIZE] = {
    {-1, 0, 1},
    {-2, 0, 2},
    {-1, 0, 1}
};

const int8_t sobel_y[KERNEL_SIZE][KERNEL_SIZE] = {
    {-1, -2, -1},
    {0, 0, 0},
    {1, 2, 1}
};

// ==================== GLOBAL BUFFERS ====================
volatile uint8_t raw_image[IMAGE_SIZE];
volatile uint8_t processed_image[IMAGE_SIZE];
volatile uint8_t edge_map[IMAGE_SIZE];
volatile uint16_t corner_scores[IMAGE_SIZE];
volatile FeatureType detected_features = NO_FEATURE;
volatile uint8_t feature_count = 0;
volatile uint8_t motion_level = 0;

// Previous frame for motion detection
volatile uint8_t prev_image[IMAGE_SIZE];

// ==================== MAIN FUNCTIONS ====================
void capture_image(void);
void apply_filter(FilterType filter);
void detect_features(void);
uint8_t compress_results(void);
void transmit_data(uint8_t* data, uint16_t size);
void display_image(uint8_t* image);

// ==================== MAIN PROGRAM ====================
int main(void) {
    printf("Image Processing Workload\n");

#ifdef W_ACCEL
    printf("Using hardware acceleration\n");
    accel_map_registers();
#endif

    // 1. Capture image from camera
    capture_image();

    // 2. Apply noise reduction (Gaussian blur)
    apply_filter(FILTER_GAUSSIAN_BLUR);

    // 3. Detect edges
    apply_filter(FILTER_EDGE_DETECT);

    // 4. Detect features
    detect_features();

    // 5. Compress results
    uint8_t compressed_size = compress_results();

    // 6. Transmit compressed data
    if (compressed_size > 0) {
        transmit_data((uint8_t*)processed_image, compressed_size);
    }

    // 7. Display processed image (optional)
    #ifdef USE_DISPLAY
    display_image((uint8_t*)processed_image);
    #endif

    // Log results
    const char* feature_names[] = {"None", "Edges", "Corners", "Pattern", "Motion"};
    printf("Features: %s (Count: %d, Motion: %d%%)\n",
           feature_names[detected_features], feature_count, motion_level);

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    return 0;
}

// ==================== IMAGE CAPTURE ====================
void capture_image(void) {
    uint8_t *camera_reg;
    uint8_t pixel;

    // Initialize camera peripheral
    periRegister(CAMERA_ID, &camera_reg);
    periInit(camera_reg);

    printf("Capturing %dx%d image...\n", IMAGE_WIDTH, IMAGE_HEIGHT);

    // Capture image row by row
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            camCapturePixel(&pixel, camera_reg, x, y);
            raw_image[y * IMAGE_WIDTH + x] = pixel;
        }
    }

    // Check for motion by comparing with previous frame
    if (prev_image[0] != 0) {  // If we have a previous frame
        uint16_t diff_sum = 0;
        for (int i = 0; i < IMAGE_SIZE; i++) {
            int16_t diff = raw_image[i] - prev_image[i];
            diff_sum += (diff > 0) ? diff : -diff;
        }
        motion_level = (diff_sum * 100) / (IMAGE_SIZE * 255);
    }

    // Store current frame for next comparison
    for (int i = 0; i < IMAGE_SIZE; i++) {
        prev_image[i] = raw_image[i];
        processed_image[i] = raw_image[i];  // Initialize processed image
    }

    periTurnOff(camera_reg);
    periLogout(CAMERA_ID);
}

// ==================== FILTER APPLICATION ====================
void apply_filter(FilterType filter) {
#ifdef W_ACCEL
    // Use hardware accelerator for convolution
    switch(filter) {
        case FILTER_GAUSSIAN_BLUR:
            accel_set_addr((uint64_t)processed_image, (uint64_t)processed_image,
                          IMAGE_SIZE, FILTER_GAUSSIAN);
            break;
        case FILTER_EDGE_DETECT:
            accel_set_addr((uint64_t)processed_image, (uint64_t)edge_map,
                          IMAGE_SIZE, FILTER_EDGE);
            break;
        default:
            return;
    }
    accel_start();
    accel_wait();

#else
    // Software implementation of convolution
    uint8_t temp_buffer[IMAGE_SIZE];
    int kernel_offset = KERNEL_SIZE / 2;

    for (int y = kernel_offset; y < IMAGE_HEIGHT - kernel_offset; y++) {
        for (int x = kernel_offset; x < IMAGE_WIDTH - kernel_offset; x++) {
            int32_t sum = 0;

            // Apply kernel
            for (int ky = -kernel_offset; ky <= kernel_offset; ky++) {
                for (int kx = -kernel_offset; kx <= kernel_offset; kx++) {
                    uint8_t pixel = processed_image[(y + ky) * IMAGE_WIDTH + (x + kx)];

                    switch(filter) {
                        case FILTER_GAUSSIAN_BLUR:
                            sum += pixel * gaussian_kernel[ky + kernel_offset][kx + kernel_offset];
                            break;
                        case FILTER_EDGE_DETECT: {
                            // Calculate gradient using Sobel
                            int32_t gx = 0, gy = 0;
                            for (int i = -1; i <= 1; i++) {
                                for (int j = -1; j <= 1; j++) {
                                    uint8_t p = processed_image[(y + i) * IMAGE_WIDTH + (x + j)];
                                    gx += p * sobel_x[i + 1][j + 1];
                                    gy += p * sobel_y[i + 1][j + 1];
                                }
                            }
                            sum = abs(gx) + abs(gy);
                            break;
                        }
                        default:
                            sum = pixel;
                    }
                }
            }

            // Normalize result
            if (filter == FILTER_GAUSSIAN_BLUR) {
                sum = sum / 16;  // Gaussian kernel normalization
            } else if (filter == FILTER_EDGE_DETECT) {
                sum = (sum > EDGE_THRESHOLD) ? 255 : 0;
            }

            // Clamp to 0-255
            if (sum > 255) sum = 255;
            if (sum < 0) sum = 0;

            temp_buffer[y * IMAGE_WIDTH + x] = (uint8_t)sum;
        }
    }

    // Copy back to processed_image
    for (int i = 0; i < IMAGE_SIZE; i++) {
        if (filter == FILTER_EDGE_DETECT) {
            edge_map[i] = temp_buffer[i];
        } else {
            processed_image[i] = temp_buffer[i];
        }
    }
#endif
}

// ==================== FEATURE DETECTION ====================
void detect_features(void) {
    uint16_t edge_count = 0;
    uint16_t corner_count = 0;

    // Simple edge detection from edge map
    for (int i = 0; i < IMAGE_SIZE; i++) {
        if (edge_map[i] > 0) {
            edge_count++;
        }
    }

    // Simple corner detection (Harris corner approximation)
    for (int y = 1; y < IMAGE_HEIGHT - 1; y++) {
        for (int x = 1; x < IMAGE_WIDTH - 1; x++) {
            uint8_t center = processed_image[y * IMAGE_WIDTH + x];

            // Check 8 neighbors
            uint8_t neighbors[8];
            neighbors[0] = processed_image[(y-1)*IMAGE_WIDTH + (x-1)];
            neighbors[1] = processed_image[(y-1)*IMAGE_WIDTH + x];
            neighbors[2] = processed_image[(y-1)*IMAGE_WIDTH + (x+1)];
            neighbors[3] = processed_image[y*IMAGE_WIDTH + (x-1)];
            neighbors[4] = processed_image[y*IMAGE_WIDTH + (x+1)];
            neighbors[5] = processed_image[(y+1)*IMAGE_WIDTH + (x-1)];
            neighbors[6] = processed_image[(y+1)*IMAGE_WIDTH + x];
            neighbors[7] = processed_image[(y+1)*IMAGE_WIDTH + (x+1)];

            // Count significant differences
            uint8_t diff_count = 0;
            for (int n = 0; n < 8; n++) {
                if (abs((int16_t)center - (int16_t)neighbors[n]) > NOISE_THRESHOLD) {
                    diff_count++;
                }
            }

            // Potential corner if many neighbors are different
            if (diff_count >= 6) {
                corner_scores[y * IMAGE_WIDTH + x] = diff_count;
                corner_count++;
            }
        }
    }

    // Determine primary feature
    if (motion_level > 30) {
        detected_features = MOTION_DETECTED;
        feature_count = motion_level;
    } else if (corner_count > 100) {
        detected_features = CORNERS_DETECTED;
        feature_count = corner_count;
    } else if (edge_count > 500) {
        detected_features = EDGES_DETECTED;
        feature_count = edge_count;
    } else {
        detected_features = NO_FEATURE;
        feature_count = 0;
    }
}

// ==================== DATA COMPRESSION ====================
uint8_t compress_results(void) {
    // Simple run-length encoding for edge map
    uint8_t compressed[IMAGE_SIZE / 2];  // Worst case compression buffer
    uint16_t comp_index = 0;
    uint8_t current_val = edge_map[0];
    uint8_t run_length = 1;

    for (int i = 1; i < IMAGE_SIZE; i++) {
        if (edge_map[i] == current_val && run_length < 255) {
            run_length++;
        } else {
            compressed[comp_index++] = current_val;
            compressed[comp_index++] = run_length;
            current_val = edge_map[i];
            run_length = 1;

            if (comp_index >= sizeof(compressed) - 2) {
                break;  // Buffer full
            }
        }
    }

    // Store last run
    if (comp_index < sizeof(compressed) - 1) {
        compressed[comp_index++] = current_val;
        compressed[comp_index++] = run_length;
    }

    printf("Compressed from %d to %d bytes\n", IMAGE_SIZE, comp_index);
    return comp_index;
}

// ==================== DATA TRANSMISSION ====================
void transmit_data(uint8_t* data, uint16_t size) {
    printf("Transmitting %d bytes...\n", size);

    uint8_t *rf_reg;
    periRegister(RF_ID, &rf_reg);
    periInit(rf_reg);

    // Send header
    uint8_t header[4];
    header[0] = (uint8_t)detected_features;
    header[1] = feature_count;
    header[2] = motion_level;
    header[3] = header[0] ^ header[1] ^ header[2];  // Checksum

    for (int i = 0; i < 4; i++) {
        rfTransmitByte(rf_reg, header[i]);
        DelayMS(5);
    }

    // Send compressed data in chunks
    for (int i = 0; i < size; i++) {
        rfTransmitByte(rf_reg, data[i]);
        if (i % 16 == 0) DelayMS(10);  // Brief pause every 16 bytes
    }

    periTurnOff(rf_reg);
    periLogout(RF_ID);
}

// ==================== DISPLAY OUTPUT ====================
#ifdef USE_DISPLAY
void display_image(uint8_t* image) {
    uint8_t *display_reg;
    periRegister(DISPLAY_ID, &display_reg);
    periInit(display_reg);

    printf("Displaying processed image...\n");

    // Send image to display
    for (int y = 0; y < IMAGE_HEIGHT; y++) {
        for (int x = 0; x < IMAGE_WIDTH; x++) {
            dispWritePixel(display_reg, x, y, image[y * IMAGE_WIDTH + x]);
        }
        DelayMS(5);  // Row delay for display refresh
    }

    periTurnOff(display_reg);
    periLogout(DISPLAY_ID);
}
#endif
