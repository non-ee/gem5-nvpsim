#include "accel_reg.h"
#include "delay.h"
#include "peripheral.h"
#include <stdint.h>

#define COUNT 10

volatile uint8_t src_array[COUNT];
volatile uint8_t dst_array[COUNT];

void sensing_task() {
    // Implement sensing task here
    uint8_t tmp;
    uint8_t *tmp_reg;

    periRegister(TMP_SENSOR_ID, &tmp_reg);
    periInit(tmp_reg);

    for (int i = 0; i < COUNT; i++) {
        tmpSense(&tmp, tmp_reg);
        src_array[i] = tmp;
        DelayMS(10);
    }

    periLogout(TMP_SENSOR_ID);
}

void pre_compute() {
    for (int i = 0; i < COUNT; i++) {
        src_array[i] -= 20;  // assume temp baseline = 20°C
    }
}

void post_compute() {
    uint32_t checksum = 0;
    for (int i = 0; i < COUNT; i++)
        checksum += dst_array[i];
    dst_array[0] = checksum & 0xFF;
}

void heavy_compute() {
#ifdef W_ACCEL
    accel_set_addr((uint64_t)src_array, (uint64_t)dst_array, COUNT);
    accel_start();
#else
    for (int i = 0; i < COUNT; i++) {
        uint32_t x = src_array[i];

        // Simulate accelerator-like workload:
        for (int j = 0; j < 100; j++) {
            x = (x * 17 + j) % 256;
        }

        dst_array[i] = x;
    }
#endif
}

void display_output() {
    printf("dst_array[0] = %d\n", dst_array[0]);
}

int main() {

    uint8_t *measure_reg;
    periRegister(MEASURE_UNIT_ID, &measure_reg);

#ifdef W_ACCEL
    accel_map_registers();
#endif

    *measure_reg = 0x1;
    sensing_task();
    *measure_reg = 0x2;

    *measure_reg = 0x1;
    pre_compute();
    heavy_compute();
    post_compute();
    *measure_reg = 0x2;

    display_output();

#ifdef W_ACCEL
    accel_unmap_registers();
#endif

    periLogout(MEASURE_UNIT_ID);

    return 0;
}
