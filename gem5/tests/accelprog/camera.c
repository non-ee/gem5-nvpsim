#include "peripheral.h"
#include <math.h>

#define IMAGE_WIDTH 8
#define IMAGE_HEIGHT 8

// ==================== CAMERA SIMULATOR FUNCTIONS ====================
// Exact same pattern as micSense, but for camera
void camCapturePixel(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y) {
    // Same virtual device protocol as micSense
    if (!(*camera_reg & VDEV_READY))
        periInit(camera_reg);

    *camera_reg = VDEV_EXEC;
    while(!(*camera_reg & VDEV_FINISH));

    // Generate test patterns using static variables (like micSense)
    static uint32_t frame_counter = 0;
    static uint8_t animation_phase = 0;
    static uint8_t test_pattern = 0;

    frame_counter++;

    // Different test patterns (like micSense's different animal sounds)
    test_pattern = (frame_counter / 100) % 4; // Change pattern every 100 frames

    switch(test_pattern) {
        case 0: // Checkerboard pattern
            *pixel = ((x / 8 + y / 8) % 2 == 0) ? 200 : 50;
            break;

        case 1: // Gradient pattern
            *pixel = (uint8_t)((x * 255) / IMAGE_WIDTH);
            break;

        case 2: // Circular pattern
            {
                int16_t dx = x - IMAGE_WIDTH/2;
                int16_t dy = y - IMAGE_HEIGHT/2;
                uint16_t dist = dx*dx + dy*dy;
                *pixel = (dist < 400) ? 255 : 100;
            }
            break;

        case 3: // Moving object pattern
            {
                animation_phase = (frame_counter / 10) % IMAGE_WIDTH;
                int16_t obj_x = animation_phase;
                int16_t obj_y = IMAGE_HEIGHT/2;
                int16_t dx = x - obj_x;
                int16_t dy = y - obj_y;
                uint16_t dist = dx*dx + dy*dy;
                *pixel = (dist < 100) ? 255 : 128;
            }
            break;

        default:
            *pixel = 128; // Gray
    }

    // Add some noise (like micSense's background noise)
    static uint32_t noise_seed = 12345;
    noise_seed = noise_seed * 1103515245 + 12345;
    uint8_t noise = (noise_seed >> 16) & 0x1F; // 5-bit noise
    *pixel = (*pixel + noise - 16) & 0xFF; // Add noise and clamp
}

// Alternative: More sophisticated simulator with different scenes
void camCapturePixelScene(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y) {
    // Same virtual device interface
    if (!(*camera_reg & VDEV_READY))
        periInit(camera_reg);

    *camera_reg = VDEV_EXEC;
    while(!(*camera_reg & VDEV_FINISH));

    // Simulate different scenes (like micSense's different animals)
    static uint32_t scene_counter = 0;
    static uint8_t current_scene = 0;
    static uint8_t object_pos = 0;

    scene_counter++;
    current_scene = (scene_counter / 500) % 3; // Change scene every 500 frames
    object_pos = (scene_counter / 10) % IMAGE_WIDTH;

    switch(current_scene) {
        case 0: // Scene 1: Indoor environment
            {
                // Wall with window
                uint8_t base = 150;
                if (x > 20 && x < 44 && y > 20 && y < 44) {
                    // Window
                    base = 230;
                    if ((x - 20) % 8 < 4 && (y - 20) % 8 < 4) {
                        base = 100; // Window frame
                    }
                }
                // Moving person
                if (abs(x - object_pos) < 4 && y > 40 && y < 56) {
                    base = 50; // Dark object
                }
                *pixel = base;
            }
            break;

        case 1: // Scene 2: Outdoor with horizon
            {
                uint8_t base;
                if (y < IMAGE_HEIGHT/2) {
                    base = 180 - (y * 60) / (IMAGE_HEIGHT/2); // Sky gradient
                } else {
                    base = 80 + ((y - IMAGE_HEIGHT/2) * 40) / (IMAGE_HEIGHT/2); // Ground gradient
                }
                // Moving object (bird/animal)
                int16_t obj_y = IMAGE_HEIGHT/3 + (sin(object_pos * 0.1) * 10);
                if (abs(x - object_pos) < 3 && abs(y - obj_y) < 3) {
                    base = 30; // Dark moving object
                }
                *pixel = base;
            }
            break;

        case 2: // Scene 3: Industrial pattern
            {
                // Grid pattern
                uint8_t base = ((x/16 + y/16) % 2 == 0) ? 170 : 90;
                // Moving robot/object
                if (abs(x - object_pos) < 6) {
                    uint8_t arm_height = 10 + abs(sin(object_pos * 0.2) * 8);
                    if (y > IMAGE_HEIGHT/2 - arm_height && y < IMAGE_HEIGHT/2 + arm_height) {
                        base = 40; // Robot arm
                    }
                }
                *pixel = base;
            }
            break;
    }

    // Add sensor noise
    static uint32_t noise_state = 1;
    noise_state = noise_state * 1664525 + 1013904223;
    *pixel ^= (noise_state >> 24) & 0x0F; // Add 4-bit noise
}

// Simple version matching micSense exactly
void camCapturePixelSimple(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y) {
    // Line-for-line match with micSense
    if (!(*camera_reg & VDEV_READY))
        periInit(camera_reg);

    *camera_reg = VDEV_EXEC;
    while(!(*camera_reg & VDEV_FINISH));

    // Static counter like micSense
    static uint32_t counter = 0;
    counter++;

    // Generate different patterns based on counter (like micSense's animal sounds)
    uint32_t pattern = counter % 100;

    if (pattern < 30) {
        // Pattern 1: Bright object moving
        int16_t pos = (counter / 2) % IMAGE_WIDTH;
        if (abs(x - pos) < 4 && abs(y - IMAGE_HEIGHT/2) < 4) {
            *pixel = 255;
        } else {
            *pixel = 100 + (x % 56);
        }
    } else if (pattern < 60) {
        // Pattern 2: Checkerboard with noise
        uint8_t check = ((x / 4 + y / 4) % 2) ? 180 : 60;
        *pixel = check + (counter % 20) - 10;
    } else if (pattern < 90) {
        // Pattern 3: Gradient with spot
        uint8_t grad = (x * 200) / IMAGE_WIDTH;
        int16_t spot_x = IMAGE_WIDTH/3 + sin(counter * 0.1) * 20;
        int16_t spot_y = IMAGE_HEIGHT/3 + cos(counter * 0.1) * 20;
        if (abs(x - spot_x) < 5 && abs(y - spot_y) < 5) {
            *pixel = 255;
        } else {
            *pixel = grad;
        }
    } else {
        // Pattern 4: Random noise (like micSense's background)
        *pixel = 128 + (counter % 64) - 32;
    }
}

// Test pattern generator for calibration
void camCapturePixelTest(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y) {
    // Virtual device protocol
    if (!(*camera_reg & VDEV_READY))
        periInit(camera_reg);

    *camera_reg = VDEV_EXEC;
    while(!(*camera_reg & VDEV_FINISH));

    // Standard test patterns
    static uint8_t test_mode = 0;
    static uint32_t frame = 0;
    frame++;

    // Cycle through test patterns every 300 frames
    test_mode = (frame / 300) % 6;

    switch(test_mode) {
        case 0: // All white
            *pixel = 255;
            break;
        case 1: // All black
            *pixel = 0;
            break;
        case 2: // Grayscale gradient
            *pixel = (x * 256) / IMAGE_WIDTH;
            break;
        case 3: // Vertical stripes
            *pixel = (x % 16 < 8) ? 255 : 0;
            break;
        case 4: // Horizontal stripes
            *pixel = (y % 16 < 8) ? 255 : 0;
            break;
        case 5: // Checkerboard
            *pixel = ((x/8 + y/8) % 2) ? 255 : 0;
            break;
    }
}
