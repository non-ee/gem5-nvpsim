accel_test: accel_reg.o peripheral.o accel_test.c
	aarch64-linux-gnu-gcc-5 -o accel_test accel_test.c peripheral.o accel_reg.o -static

accel_reg.o: accel_reg.c accel_reg.h
	aarch64-linux-gnu-gcc-5 -c accel_reg.c

peripheral.o: peripheral.c peripheral.h
	aarch64-linux-gnu-gcc-5 -c peripheral.c

clean:
	rm accel_test peripheral.o
