#include "accel_reg.h"
#include "peripheral.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/mman.h>

#define COUNT 10

volatile uint8_t src_array[COUNT];
volatile uint8_t dst_array[COUNT];

void test_tmp_sensor() {
    uint8_t tmp;
    uint8_t* tmp_reg;

    periRegister(TMP_SENSOR_ID, &tmp_reg);

    for (int i = 0; i < COUNT; i++) {
        periInit(tmp_reg);
        tmpSense(&tmp, tmp_reg);
    }


    periLogout(TMP_SENSOR_ID);
}

void test_cmd_reg () {
    *cmd_reg |= ACCEL_CMD_START;

    printf("Started accelerator\n");

    // while (!(*cmd_reg & (1 << 3 )));
    while (!(*cmd_reg & ACCEL_CMD_DONE));

    munmap(cmd_reg, sizeof(uint8_t));
    printf("Finished test\n");
}

void test_addr_reg() {
    printf("Started accelerator\n");

    *src_reg = (uint64_t)src_array;
    *dst_reg = (uint64_t)dst_array;
    *cmd_reg |= ACCEL_CMD_START;

    // printf("Read from address %p\n", src_array);
    // printf("Write to address %p\n", dst_array);

    while (!(*cmd_reg & ACCEL_CMD_DONE));

    munmap(cmd_reg, sizeof(uint8_t));
    munmap(src_reg, sizeof(uint32_t));

    printf("Finished test\n");
}

int main() {
    accel_map_registers();
    test_tmp_sensor();
    // test_addr_reg();

    return 0;
}
