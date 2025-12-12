//
//
#ifndef __ACCEL_REG_H__
#define __ACCEL_REG_H__

#include <stdint.h>

#define ACCEL_BASE_ADDR 0x40000000
#define ACCEL_CMD_REG 0x00
#define ACCEL_SRC_REG 0x08
#define ACCEL_DST_REG 0x10
#define ACCEL_COUNT_REG 0x18

#define SRC_PA 0x90000000
#define DST_PA 0x90001000

extern uint8_t *accel;
extern uint8_t *cmd_reg;
extern uint64_t *src_reg;
extern uint64_t *dst_reg;


enum AccelState {
    IDLE = 0,
    START = 1,
    INIT = 2,
    DMA_READ = 3,
    DMA_WRITE = 4,
    COMPUTE = 5,
    CPU_INT = 6,
    DONE = 7
};

void accel_map_registers();
void accel_unmap_registers();

void accel_set_addr(uint64_t src_addr, uint64_t dst_addr, uint32_t count);
void accel_start();

#endif
