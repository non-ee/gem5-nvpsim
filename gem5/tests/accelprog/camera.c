#include "peripheral.h"
#include <math.h>

#define IMAGE_WIDTH 8
#define IMAGE_HEIGHT 8

// ==================== CAMERA SIMULATOR FUNCTIONS ====================
// Exact same pattern as micSense, but for camera
void camCapturePixel(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y) {
    // Same virtual device protocol as micSense
    if (!(*camera_reg & VDEV_READY && *camera_reg & ~VDEV_CHAOS))
        periInit(camera_reg);

    while (*camera_reg & VDEV_BUSY);

    *camera_reg = VDEV_EXEC;
    while(!(*camera_reg & VDEV_FINISH));
}
