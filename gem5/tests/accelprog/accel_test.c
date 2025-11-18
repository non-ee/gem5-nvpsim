#include "peripheral.h"
#include <stdio.h>
#include <stdint.h>

#define COUNT 10
uint8_t *sensor_mem;
uint8_t *output_mem;

#define ACCEL_BASE_ADDR PERI_ADDR[2]
#define ACCEL_CMD_REG 0x00
#define ACCEL_SRC_ADDR 0x04
#define ACCEL_DST_ADDR 0x08

#define ACCEL_REG(offset) (*(volatile uint32_t*)(ACCEL_BASE_ADDR + (offset)))

#define ACCEL_CMD_START 0x01
#define ACCEL_CMD_ABORT 0x02

void sensing_task() {
    uint8_t tmp, rf;
    uint8_t* tmp_reg, *rf_reg;

    periRegister(TMP_SENSOR_ID, &tmp_reg);
    periRegister(RF_ID, &rf_reg);
    printf("Registers initialized\n");

    periInit(tmp_reg);
    periInit(rf_reg);

    for (int i = 0; i < COUNT; i++) {
        tmpSense(&tmp, tmp_reg);
        // sensor_mem[i] = tmp;
        tmpSense(&rf, rf_reg);
    }

    periLogout(TMP_SENSOR_ID);
    periLogout(RF_ID);
}

void accel_task() {
    // Write into Accelerator registers
    ACCEL_REG(ACCEL_SRC_ADDR) = (uint32_t)(uintptr_t)sensor_mem;
    ACCEL_REG(ACCEL_DST_ADDR) = (uint32_t)(uintptr_t)output_mem;
    ACCEL_REG(ACCEL_CMD_REG) = ACCEL_CMD_START;
}

int main() {
    sensor_mem = mmap(NULL, COUNT,
                      PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS,
                      -1, 0);

    output_mem = mmap(NULL, COUNT,
                      PROT_READ | PROT_WRITE,
                      MAP_PRIVATE | MAP_ANONYMOUS,
                      -1, 0);

    sensing_task();
    // accel_task();

    munmap(sensor_mem, COUNT);
    munmap(output_mem, COUNT);

    return 0;
}
