accel_test: peripheral.o accel_test.c
	aarch64-linux-gnu-gcc-5 -o accel_test accel_test.c peripheral.o -static

peripheral.o: peripheral.c peripheral.h
	aarch64-linux-gnu-gcc-5 -c peripheral.c

clean:
	rm accel_test peripheral.o
