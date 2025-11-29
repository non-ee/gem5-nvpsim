#include "accel_reg.h"
#include "delay.h"
#include "peripheral.h"
#include <stdint.h>

#define COUNT 10

volatile uint8_t src_array[COUNT];
volatile uint8_t dst_array[COUNT];

// uint8_t *src_array;
// uint8_t *dst_array;

void sensing_task() {
    // Implement sensing task here
    uint8_t tmp;
    uint8_t *tmp_reg;

    periRegister(TMP_SENSOR_ID, &tmp_reg);

    for (int i = 0; i < COUNT; i++) {
        periInit(tmp_reg);
        tmpSense(&tmp, tmp_reg);
        src_array[i] = 30;
        DelayMS(10);
    }

    periLogout(TMP_SENSOR_ID);
}

void vector_add() {
    for (int i = 0; i < COUNT; i++) {
        dst_array[i] = src_array[i] * 2 + 3;
    }
}

void heavy_task() {
    // Phase 1: normalize
    for (int i = 0; i < COUNT; i++)
        src_array[i] -= 20;  // assume temp baseline = 20°C

    // Phase 2: heavy loop
    accel_set_addr((uint64_t)src_array, (uint64_t)dst_array);
    accel_start();

    // Phase 3: reduce output
    uint32_t checksum = 0;
    for (int i = 0; i < COUNT; i++)
        checksum += dst_array[i];

    dst_array[0] = checksum & 0xFF;
}

void display_output() {
    printf("dst_array[0] = %d\n", dst_array[0]);
}

void tasks() {
    accel_map_registers();
    sensing_task();
    heavy_task();
    display_output();
    accel_unmap_registers();
}

int main() {

    tasks();

    return 0;
}
