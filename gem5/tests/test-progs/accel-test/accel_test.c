#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/mman.h>
#include <stdint.h>
#include "reg51.h"
#include "delay.h"
#include "fourier.h"
#include "peripheral.h"
#include "encode.h"

#define ANum 100    // acceleration collection number in one packet
#define PeriNum 3

// --- Function prototypes ---
void sensing_task(uint8_t *tmp_reg, uint8_t *acc_reg, uint8_t *collection);
void computing_task(uint8_t *collection, uint8_t *payload, int *payload_size);
void comm_task(uint8_t *rf_reg, uint8_t *payload, int payload_size);


// ============================================================
// Main Function
// ============================================================
int main()
{
    printf("Program Start.\n");

    // Peripheral register mapping
    uint8_t *tmp_reg = (uint8_t*) mmap(PERI_ADDR[TMP_SENSOR_ID], sizeof(uint8_t),
                            PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_PRIVATE, -1, 0);
    uint8_t *acc_reg = (uint8_t*) mmap(PERI_ADDR[ACC_SENSOR_ID], sizeof(uint8_t),
                            PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_PRIVATE, -1, 0);
    uint8_t *rf_reg  = (uint8_t*) mmap(PERI_ADDR[RF_ID], sizeof(uint8_t),
                            PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_PRIVATE, -1, 0);

    printf("Peripherals Registered.\n");

    uint8_t collection[1 + 2 + 6 * ANum];
    uint8_t payload[256];   // assumed size, adjust as needed
    int payload_size = 0;
    int count = 0;

    printf("Start Execution.\n");

    for (int j = 0; j < PeriNum / 2; j++) {
        sensing_task(tmp_reg, acc_reg, collection);
        computing_task(collection, payload, &payload_size);
        comm_task(rf_reg, payload, payload_size);
        printf("Yeah! Complete the %d-th packet.\n", ++count);
    }

    // Peripheral logout
    periLogout(TMP_SENSOR_ID);
    periLogout(ACC_SENSOR_ID);
    periLogout(RF_ID);

    printf("Program End.\n");
    return 0;
}


// ============================================================
// Function Implementations
// ============================================================

void sensing_task(uint8_t *tmp_reg, uint8_t *acc_reg, uint8_t *collection)
{
    int tmp, acc_x, acc_y, acc_z;

    // TMP Sensor
    periInit(tmp_reg);
    tmpSense(&tmp, tmp_reg);
    collection[0] = (tmp >> 8) & 0xFF;
    collection[1] = tmp & 0xFF;
    printf("--Temperature: %d°C collected.\n", tmp);

    // ACC Sensor
    periInit(acc_reg);
    for (int i = 1; i <= 10; i++) {
        accSense(&acc_x, &acc_y, &acc_z, acc_reg);
        collection[6*i-4] = (acc_x >> 8) & 0xFF;
        collection[6*i-3] = acc_x & 0xFF;
        collection[6*i-2] = (acc_y >> 8) & 0xFF;
        collection[6*i-1] = acc_y & 0xFF;
        collection[6*i]   = (acc_z >> 8) & 0xFF;
        collection[6*i+1] = acc_z & 0xFF;
        printf("--Acceleration #%d collected.\n", i);
    }
}


void computing_task(uint8_t *collection, uint8_t *payload, int *payload_size)
{
    uint8_t ImagIn[16]  = {0};
    uint8_t ImagOut[16] = {0};
    uint8_t RealOut[16] = {0};

    // Perform FFT
    fft(32, 0, collection, ImagIn, RealOut, payload);
    printf("FFT completed.\n");

    // Encode the data
    getEncodedPacket(payload, payload_size);
    printf("Encoded packet size: %d Bytes.\n", *payload_size);
}


void comm_task(uint8_t *rf_reg, uint8_t *payload, int payload_size)
{
    // Initialize Zigbee
    periInit(rf_reg);
    // Transmit data
    rfTrans(rf_reg, payload);
    printf("RF transmission completed.\n");
}
