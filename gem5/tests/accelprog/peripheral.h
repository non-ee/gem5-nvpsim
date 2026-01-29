//
//
#ifndef __PERIPHERAL_H__
#define __PERIPHERAL_H__

#include <stdio.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <stdint.h>

#define VDEV_INIT		0x80
#define VDEV_EXEC		0x40
#define VDEV_TURNOFF    0x20
#define VDEV_READY	    0x04
#define VDEV_FINISH 	0x01

static void * PERI_ADDR[3] = {
	(void *) 0x3e800000,
	(void *) 0x3e900000,
	(void *) 0x3ea00000
};

/** Accelerometer Samples **/
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} AccelSample;

// Static sample data array with 100 samples
static const AccelSample accel_samples[] = {
    // Group 1-10: Stationary/calibration (gravity on Z axis)
    {0, 0, 1000},    // 1. Stationary, flat surface
    {1, -2, 998},    // 2. Slight noise
    {-1, 1, 1002},   // 3. Slight noise
    {2, 0, 999},     // 4. Slight noise
    {0, 2, 1001},    // 5. Slight noise
    {-2, -1, 997},   // 6. Slight noise
    {1, 1, 1000},    // 7. Stationary
    {0, -1, 999},    // 8. Slight noise
    {-1, 0, 1001},   // 9. Slight noise
    {1, -1, 998},    // 10. Slight noise

    // Group 11-20: Gentle tilting forward (X-axis changes)
    {100, 10, 990},  // 11. Start forward tilt
    {150, 15, 980},  // 12. More tilt
    {200, 20, 970},  // 13. More tilt
    {250, 25, 960},  // 14. More tilt
    {300, 30, 950},  // 15. Forward tilt
    {250, 25, 960},  // 16. Less tilt
    {200, 20, 970},  // 17. Less tilt
    {150, 15, 980},  // 18. Less tilt
    {100, 10, 990},  // 19. Less tilt
    {50, 5, 995},    // 20. Almost level

    // Group 21-30: Gentle tilting sideways (Y-axis changes)
    {10, 100, 990},  // 21. Start right tilt
    {15, 150, 980},  // 22. More tilt
    {20, 200, 970},  // 23. More tilt
    {25, 250, 960},  // 24. More tilt
    {30, 300, 950},  // 25. Right tilt
    {25, 250, 960},  // 26. Less tilt
    {20, 200, 970},  // 27. Less tilt
    {15, 150, 980},  // 28. Less tilt
    {10, 100, 990},  // 29. Less tilt
    {5, 50, 995},    // 30. Almost level

    // Group 31-40: Shaking motion
    {200, -150, 850},  // 31. Shake right-forward
    {-180, 120, 870},  // 32. Shake left-back
    {220, -130, 830},  // 33. Shake right-forward
    {-200, 140, 860},  // 34. Shake left-back
    {180, -120, 840},  // 35. Shake right-forward
    {-160, 110, 880},  // 36. Shake left-back
    {190, -140, 820},  // 37. Shake right-forward
    {-170, 130, 850},  // 38. Shake left-back
    {210, -110, 830},  // 39. Shake right-forward
    {-190, 150, 860},  // 40. Shake left-back

    // Group 41-50: Free fall simulation (brief weightlessness)
    {0, 0, 0},        // 41. Free fall start
    {5, 5, 10},       // 42. Slight air resistance
    {-5, -5, 5},      // 43. Turbulence
    {10, -10, 15},    // 44. Tumbling
    {-10, 10, 8},     // 45. Tumbling
    {0, 0, 20},       // 46. Stabilizing
    {0, 0, 50},       // 47. Stabilizing
    {0, 0, 200},      // 48. Stabilizing
    {0, 0, 600},      // 49. Recovering
    {0, 0, 900},      // 50. Almost recovered

    // Group 51-60: Circular motion (device rotation)
    {707, 707, 0},    // 51. 45° tilt
    {1000, 0, 0},     // 52. On side
    {707, -707, 0},   // 53. 135° tilt
    {0, -1000, 0},    // 54. On other side
    {-707, -707, 0},  // 55. 225° tilt
    {-1000, 0, 0},    // 56. Upside down
    {-707, 707, 0},   // 57. 315° tilt
    {0, 1000, 0},     // 58. On side
    {707, 707, 0},    // 59. Back to 45°
    {0, 0, 1000},     // 60. Back to level

    // Group 61-70: Vibration pattern
    {50, 0, 950},     // 61. Small vibration
    {-50, 0, 950},    // 62.
    {50, 0, 950},     // 63.
    {-50, 0, 950},    // 64.
    {0, 50, 950},     // 65.
    {0, -50, 950},    // 66.
    {0, 50, 950},     // 67.
    {0, -50, 950},    // 68.
    {30, 30, 940},    // 69. Diagonal vibration
    {-30, -30, 940},  // 70.

    // Group 71-80: Sudden movement/impact
    {0, 0, 1000},     // 71. Normal
    {800, 0, 600},    // 72. Sudden forward jerk
    {1200, 0, 200},   // 73. Impact
    {400, 0, 800},    // 74. Recoil
    {100, 0, 950},    // 75. Settling
    {0, 800, 600},    // 76. Side impact
    {0, 1200, 200},   // 77. Side impact peak
    {0, 400, 800},    // 78. Recoil
    {0, 100, 950},    // 79. Settling
    {0, 0, 1000},     // 80. Back to normal

    // Group 81-90: Walking motion simulation
    {200, 50, 970},   // 81. Step forward
    {-100, -20, 980}, // 82. Foot lift
    {150, 40, 960},   // 83. Step forward
    {-80, -30, 970},  // 84. Foot lift
    {180, 60, 950},   // 85. Step forward
    {-90, -10, 980},  // 86. Foot lift
    {190, 30, 940},   // 87. Step forward
    {-70, -40, 960},  // 88. Foot lift
    {170, 20, 970},   // 89. Step forward
    {-60, -30, 990},  // 90. Foot lift

    // Group 91-100: Return to stable with variations
    {0, 0, 1000},     // 91. Stable
    {20, -10, 990},   // 92. Minor adjustment
    {-10, 20, 995},   // 93. Minor adjustment
    {15, 15, 980},    // 94. Slight movement
    {-15, -15, 985},  // 95. Slight movement
    {10, -5, 990},    // 96. Settling
    {-5, 10, 995},    // 97. Settling
    {5, 5, 998},      // 98. Almost stable
    {-2, 2, 999},     // 99. Almost stable
    {0, 0, 1000}      // 100. Completely stable
};

static uint8_t sample_index = 0;

void	periRegister(int peri_id, uint8_t **reg_file);
void	periLogout(int peri_id);
void	periInit(uint8_t *cmd_reg);
void    periTurnOff(uint8_t *cmd_reg);

void	tmpSense(uint8_t *tmp, uint8_t *cmd_reg);
void    accelSense(int16_t *x, int16_t *y, int16_t *z, uint8_t *reg);
void    micSense(uint16_t *sample, uint8_t *reg);

// camera
void    camCapturePixel(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y);
void    camCapturePixelScene(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y);
void    camCapturePixelSimple(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y);
void    camCapturePixelTest(uint8_t *pixel, uint8_t *camera_reg, uint16_t x, uint16_t y);

void	rfTrans(uint8_t *cmd_reg);
void    rfTransmitByte(uint8_t *cmd_reg, uint8_t data);

void	generalVdevActive(uint8_t *cmd_reg);

#endif
