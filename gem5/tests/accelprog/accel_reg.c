#include <sys/mman.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "accel_reg.h"


uint8_t *accel = NULL;
uint8_t *cmd_reg = NULL;
uint64_t *src_reg = NULL;
uint64_t *dst_reg = NULL;
uint32_t *input_count_reg = NULL;
uint32_t *output_count_reg = NULL;

void accel_map_registers() {
    accel = mmap(
        (void*)ACCEL_BASE_ADDR,
        4096,
        PROT_READ | PROT_WRITE,
        MAP_ANONYMOUS | MAP_PRIVATE,
        -1,
        0
    );

    if (accel == MAP_FAILED) {
        perror("mmap failed");
        return;
    }

    cmd_reg = accel + ACCEL_CMD_REG;
    src_reg = (uint64_t*)(accel + ACCEL_SRC_REG);
    dst_reg = (uint64_t*)(accel + ACCEL_DST_REG);
    input_count_reg = (uint32_t*)(accel + ACCEL_INPUT_COUNT_REG);
    output_count_reg = (uint32_t*)(accel + ACCEL_OUTPUT_COUNT_REG);
}

void accel_unmap_registers() {
    if (accel != NULL) {
        munmap(accel, 4096);
        accel = NULL;
    }

    accel = NULL;
    cmd_reg = NULL;
    src_reg = NULL;
    dst_reg = NULL;
    input_count_reg = NULL;
    output_count_reg = NULL;
}

void accel_set_addr(uint64_t src_addr, uint64_t dst_addr, uint32_t input_count, uint32_t output_count) {
    *src_reg = src_addr;
    *dst_reg = dst_addr;
    *input_count_reg = input_count;
    *output_count_reg = output_count;
}

void accel_start() {
    *cmd_reg = CMD_INIT;
    while (!(*cmd_reg & CMD_DONE_BIT));
}
