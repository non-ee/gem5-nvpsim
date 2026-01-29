CC := aarch64-linux-gnu-gcc-5
W_ACCEL ?= 0
CFLAGS := -Wall -Wextra -O2

ifeq ($(W_ACCEL), 1)
	CFLAGS += -DW_ACCEL
endif

image_processing: image_processing.c accel_reg.o peripheral.o delay.o camera.o
	$(CC) $(CFLAGS) -o image_processing image_processing.c peripheral.o accel_reg.o delay.o camera.o -static -lm

har: accel_reg.o peripheral.o delay.o har.c
	$(CC) $(CFLAGS) -o har har.c peripheral.o accel_reg.o delay.o -static -lm

noise_monitoring: accel_reg.o peripheral.o delay.o noise_monitoring.c
	$(CC) $(CFLAGS) -o noise_monitoring noise_monitoring.c peripheral.o accel_reg.o delay.o -static

test_w_accel: accel_reg.o peripheral.o delay.o test_w_accel.c
	$(CC) $(CFLAGS) -o test_w_accel test_w_accel.c peripheral.o accel_reg.o delay.o -static

accel_reg.o: accel_reg.c accel_reg.h
	$(CC) -c accel_reg.c

camera.o: peripheral.h camera.c
	$(CC) -c camera.c

peripheral.o: peripheral.c camera.c peripheral.h
	$(CC) -c peripheral.c

delay.o: delay.c delay.h
	$(CC) -c delay.c

clean:
	rm accel_test *.o
