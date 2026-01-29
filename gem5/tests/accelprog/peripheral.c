//
//
#include "peripheral.h"
#include "accel_reg.h"
#include <stdint.h>

void
periRegister(int peri_id, uint8_t **reg_file){
	*reg_file = (uint8_t*) mmap(
	PERI_ADDR[peri_id], sizeof(uint8_t),
	PROT_READ|PROT_WRITE, MAP_ANONYMOUS|MAP_PRIVATE, -1, 0);
}

void
periLogout(int peri_id){
	munmap(PERI_ADDR[peri_id], sizeof(uint8_t));
}

void
periInit(uint8_t *cmd_reg){
	*cmd_reg = VDEV_INIT;
	while(!(*cmd_reg & VDEV_READY));
}

void
periTurnOff(uint8_t *cmd_reg){
    *cmd_reg = VDEV_TURNOFF;
}

void
tmpSense(uint8_t *tmp, uint8_t *cmd_reg){
    // Check if the device is ready
    if (!(*cmd_reg & VDEV_READY)) {
        periInit(cmd_reg);
    }

	*cmd_reg = VDEV_EXEC;
	while(!(*cmd_reg & VDEV_FINISH));
	*tmp = 3;
}

void accelSense(int16_t *x, int16_t *y, int16_t *z, uint8_t *reg) {
    if (!(*reg & VDEV_READY)) {
        periInit(reg);
    }

	*reg = VDEV_EXEC;
	while(!(*reg & VDEV_FINISH));

	// *x = 2;
	*x = accel_samples[sample_index].x;
	*y = accel_samples[sample_index].y;
	*z = accel_samples[sample_index].z;
	sample_index++;
}

void micSense(uint16_t *sample, uint8_t *reg) {
    if (!(*reg & VDEV_READY))
        periInit(reg);

    *reg = VDEV_EXEC;
    while(!(*reg & VDEV_FINISH));

    // Simulate different animal sounds
    static uint32_t counter = 0;
    counter++;

    // Generate different patterns for different animals
    uint32_t pattern = counter % 100;

    if (pattern < 10) {
        // Bird pattern: high frequency chirps
        *sample = 800 + (counter % 200);
    } else if (pattern < 20) {
        // Mammal pattern: medium frequency
        *sample = 400 + (counter % 150);
    } else if (pattern < 25) {
        // Insect pattern: very high frequency
        *sample = 900 + (counter % 100);
    } else {
        // Background noise
        *sample = 200 + (counter % 50);
    }
}

void
rfTrans(uint8_t *cmd_reg){
	*cmd_reg = VDEV_EXEC;
	while(!(*cmd_reg & VDEV_FINISH));
};

void
rfTransmitByte(uint8_t *reg, uint8_t data){
    if (!(*reg & VDEV_READY))
        periInit(reg);
	*reg = VDEV_EXEC;
	while(!(*reg & VDEV_FINISH));
};

void
generalVdevActive(uint8_t *cmd_reg){
	*cmd_reg = VDEV_EXEC;
	while(!(*cmd_reg & VDEV_FINISH));
}
