accel_test: accel_reg.o peripheral.o delay.o accel_test.c
	aarch64-linux-gnu-gcc-5 -o accel_test accel_test.c peripheral.o accel_reg.o delay.o -static

test_wo_accel: accel_reg.o peripheral.o delay.o test_wo_accel.c
	aarch64-linux-gnu-gcc-5 -o test_wo_accel test_wo_accel.c peripheral.o accel_reg.o delay.o -static

test_w_accel: accel_reg.o peripheral.o delay.o test_w_accel.c
	aarch64-linux-gnu-gcc-5 -o test_w_accel test_w_accel.c peripheral.o accel_reg.o delay.o -static

accel_reg.o: accel_reg.c accel_reg.h
	aarch64-linux-gnu-gcc-5 -c accel_reg.c

peripheral.o: peripheral.c peripheral.h
	aarch64-linux-gnu-gcc-5 -c peripheral.c

delay.o: delay.c delay.h
	aarch64-linux-gnu-gcc-5 -c delay.c

clean:
	rm accel_test *.o
