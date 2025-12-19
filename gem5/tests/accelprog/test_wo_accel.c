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
    for (int i = 0; i < COUNT; i++) {
        uint32_t x = src_array[i];

        // Simulate accelerator-like workload:
        for (int j = 0; j < 100; j++) {
            x = (x * 17 + j) % 256;
        }

        dst_array[i] = x;
    }
}


void display_output() {
    printf("Output: %d\n", dst_array[0]);
}

int main() {

    sensing_task();

    pre_compute();
    heavy_compute();
    post_compute();

    display_output();

    return 0;
}
