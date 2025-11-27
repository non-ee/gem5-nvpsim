//
//
#ifndef __ACCEL_REG_H__
#define __ACCEL_REG_H__

#include <stdint.h>

#define ACCEL_BASE_ADDR 0x40000000
#define ACCEL_CMD_REG 0x00
#define ACCEL_SRC_REG 0x08
#define ACCEL_DST_REG 0x10

#define ACCEL_CMD_START (1 << 0)
#define ACCEL_CMD_ABORT (1 << 1)
#define ACCEL_CMD_DMA_READ (1 << 2)
#define ACCEL_CMD_DMA_WRITE (1 << 3)
#define ACCEL_CMD_COMPUTE (1 << 4)
#define ACCEL_CMD_DONE (1 << 5)

extern uint8_t *accel;
extern uint8_t *cmd_reg;
extern uint64_t *src_reg;
extern uint64_t *dst_reg;


void accel_map_registers();
void accel_unmap_registers();
#endif
