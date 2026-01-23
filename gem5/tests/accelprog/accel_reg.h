//
//
#ifndef __ACCEL_REG_H__
#define __ACCEL_REG_H__

#include <stdint.h>
#include <sys/types.h>

#define ACCEL_BASE_ADDR 0x50000000
#define ACCEL_CMD_REG 0x00
#define ACCEL_SRC_REG 0x08
#define ACCEL_DST_REG 0x10
#define ACCEL_INPUT_COUNT_REG 0x18
#define ACCEL_OUTPUT_COUNT_REG 0x20

#define CMD_INIT 0x01
#define CMD_DMA_READ 0x02
#define CMD_DMA_WRITE 0x03
#define CMD_COMPUTE 0x04
#define CMD_INTERRUPT 0x05

#define CMD_DONE_BIT (1 << 6)

extern uint8_t *accel;
extern uint8_t *cmd_reg;
extern uint64_t *src_reg;
extern uint64_t *dst_reg;
extern uint32_t *input_count_reg;
extern uint32_t *output_count_reg;


void accel_map_registers();
void accel_unmap_registers();

void accel_set_addr(uint64_t src_addr, uint64_t dst_addr, uint32_t input_count, uint32_t output_count);
void accel_start();

#endif
