#include "peripheral.h"
#include <fcntl.h>
#include <stdio.h>
#include <stdint.h>
#include <sys/mman.h>

#define COUNT 10

#define ACCEL_BASE_ADDR 0x20200000
#define ACCEL_CMD_REG 0x00
#define ACCEL_SRC_ADDR 0x04
#define ACCEL_DST_ADDR 0x08

#define ACCEL_REG(offset) (ACCEL_BASE_ADDR + (offset))

#define ACCEL_CMD_START 0x01
#define ACCEL_CMD_ABORT 0x02
#define ACCEL_CMD_DMA_READ (1 << 2)
#define ACCEL_CMD_DONE (1 << 5)

volatile uint8_t src_array[COUNT];
volatile uint8_t dst_array[COUNT];

void test_tmp_sensor() {
    uint8_t tmp;
    uint8_t* tmp_reg;

    periRegister(TMP_SENSOR_ID, &tmp_reg);
    printf("Registers initialized\n");

    periInit(tmp_reg);

    tmpSense(&tmp, tmp_reg);

    periLogout(TMP_SENSOR_ID);
}

void test_cmd_reg () {
    // Write into Accelerator registers
    uint8_t* cmd_reg;

    cmd_reg = (uint8_t *) mmap ( (void*)0x40000000, sizeof(uint8_t), PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_PRIVATE, -1, 0 );
    *cmd_reg |= ACCEL_CMD_START;

    printf("Started accelerator\n");

    // while (!(*cmd_reg & (1 << 3 )));
    while (!(*cmd_reg & ACCEL_CMD_DONE));

    munmap(cmd_reg, sizeof(uint8_t));
    printf("Finished test\n");
}

void test_addr_reg() {
    uint8_t *accel = mmap(
        (void*)0x40000000,   // page-aligned
        4096,                // whole page
        PROT_READ|PROT_WRITE,
        MAP_ANONYMOUS|MAP_PRIVATE,
        -1,
        0
    );


    uint8_t* cmd_reg = accel + 0x00;      // offset 0x0
    uint64_t* src_reg = (uint64_t*)(accel + 0x08); // offset 0x8
    uint64_t* dst_reg = (uint64_t*)(accel + 0x10); // offset 0x10

    printf("Started accelerator\n");

    *src_reg = (uint64_t)src_array;
    *dst_reg = (uint64_t)dst_array;
    *cmd_reg |= ACCEL_CMD_START;

    printf("Read from address %p\n", src_array);
    printf("Write to address %p\n", dst_array);

    while (!(*cmd_reg & ACCEL_CMD_DONE));

    munmap(cmd_reg, sizeof(uint8_t));
    munmap(src_reg, sizeof(uint32_t));

    printf("Finished test\n");
}

int main() {

    test_tmp_sensor();
    // test_cmd_reg();
    test_addr_reg();

    return 0;
}
