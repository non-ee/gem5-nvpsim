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

#define CMD_FINISH 0x00
#define CMD_INIT 0x01
#define CMD_DMA_READ 0x02
#define CMD_DMA_WRITE 0x03
#define CMD_COMPUTE 0x04
#define CMD_INTERRUPT 0x05

extern uint8_t *accel;
extern uint8_t *cmd_reg;
extern uint64_t *src_reg;
extern uint64_t *dst_reg;


void accel_map_registers();
void accel_unmap_registers();

void accel_set_addr(uint64_t src_addr, uint64_t dst_addr, uint32_t count);
void accel_start();

#endif
