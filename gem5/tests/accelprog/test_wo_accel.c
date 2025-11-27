#include "accel_reg.h"
#include "delay.h"
#include "peripheral.h"

#define COUNT 3

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
        src_array[i] = *tmp_reg;
        DelayMS(10);
    }

    periLogout(TMP_SENSOR_ID);
}

void compute_task() {
    // Implement compute task here
}

int main() {
    accel_map_registers();

    // sensing task
    sensing_task();

    // compute task

    return 0;
}
