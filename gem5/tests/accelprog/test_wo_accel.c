#include "accel_reg.h"
#include "delay.h"
#include "peripheral.h"

#define COUNT 10

volatile uint8_t src_array[COUNT];
volatile uint8_t dst_array[COUNT];

void sensing_task() {
    // Implement sensing task here
    uint8_t tmp;
    uint8_t *tmp_reg;

    periRegister(TMP_SENSOR_ID, &tmp_reg);

    for (int i = 0; i < COUNT; i++) {
        periInit(tmp_reg);
        tmpSense(&tmp, tmp_reg);
        src_array[i] = tmp;
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
    // Normalize
    for (int i = 0; i < COUNT; i++)
        src_array[i] -= 20;  // assume temp baseline = 20°C

    // Phase 2: heavy loop
    for (int i = 0; i < COUNT; i++) {
        uint32_t x = src_array[i];

        // Simulate accelerator-like workload:
        for (int j = 0; j < 100; j++) {
            x = (x * 17 + j) % 256;
        }

        dst_array[i] = x;
    }

    // Phase 3: reduce output
    uint32_t checksum = 0;
    for (int i = 0; i < COUNT; i++)
        checksum += dst_array[i];

    dst_array[0] = checksum & 0xFF;
}

void display_output() {
    printf("Output: %d\n", dst_array[0]);
}

int main() {
    accel_map_registers();

    // sensing task
    sensing_task();
    // compute task
    heavy_task();
    // display output
    display_output();

    accel_unmap_registers();

    return 0;
}
