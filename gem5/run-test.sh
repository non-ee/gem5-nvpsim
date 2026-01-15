rm m5out/batch_res.csv

#echo "========================================================="
#cap=10
#energy=4
#echo "cap: $cap; entergy: $energy"
#build/ARM/gem5.debug configs/example/test_engy_vdev.py $cap $energy
#build/ARM/gem5.debug --debug-flag=VirtualDevice,EnergyMgmt --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2

count=20
perf_boost=1
cap=10
energy=6

W_ACCEL=1

arg_prog=""

if [ $W_ACCEL -eq 1 ]; then
    arg_prog="w_accel"
else
    arg_prog="wo_accel"
fi

prog="test_w_accel"
script="configs/accel/sim_${arg_prog}.py"

sed -i "s/#define COUNT .*/#define COUNT ${count}/" tests/accelprog/${prog}.c
make ${prog} -C tests/accelprog/ W_ACCEL=$W_ACCEL

FLAG=--debug-flag=MeasureUnit,Accelerator

echo "========================================================="
echo "cap: $cap; energy: $energy"
build/ARM/gem5.debug $FLAG $script $cap $energy $prog $count $perf_boost
# build/ARM/gem5.debug --debug-flag=VirtualDevice --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2
