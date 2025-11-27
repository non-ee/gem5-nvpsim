#include <sys/mman.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include "accel_reg.h"

uint8_t *accel = NULL;
uint8_t *cmd_reg = NULL;
uint64_t *src_reg = NULL;
uint64_t *dst_reg = NULL;

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
}
