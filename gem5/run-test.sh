rm m5out/batch_res.csv

#echo "========================================================="
#cap=10
#energy=4
#echo "cap: $cap; entergy: $energy"
#build/ARM/gem5.debug configs/example/test_engy_vdev.py $cap $energy
#build/ARM/gem5.debug --debug-flag=VirtualDevice,EnergyMgmt --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2

count=10
perf_boost=5
cap=10
energy=6
prog=test_w_accel
# prog=test_wo_accel
# prog=accel_test

sed -i "s/#define COUNT .*/#define COUNT ${count}/" tests/accelprog/${prog}.c
make ${prog} -C tests/accelprog/

FLAG=--debug-flag=Accelerator

echo "========================================================="
echo "cap: $cap; energy: $energy"
build/ARM/gem5.debug $FLAG configs/accel/sim.py $cap $energy $prog $count $perf_boost
# build/ARM/gem5.debug --debug-flag=VirtualDevice --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2
