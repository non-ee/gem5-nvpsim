rm m5out/batch_res.csv

#echo "========================================================="
#cap=10
#energy=4
#echo "cap: $cap; entergy: $energy"
#build/ARM/gem5.debug configs/example/test_engy_vdev.py $cap $energy
#build/ARM/gem5.debug --debug-flag=VirtualDevice,EnergyMgmt --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2

perf_boost=1
cap=30
energy=25

W_ACCEL=0


prog="image_processing"
# prog="har"
# prog="noise_monitoring"
# prog="test_w_accel"

arg_prog=""
if [ $W_ACCEL -eq 1 ]; then
    arg_prog="sim_w_accel"
else
    arg_prog="sim_wo_accel"
fi

script="configs/accel/${prog}/${arg_prog}.py"

sed -i "s/#define COUNT .*/#define COUNT ${count}/" tests/accelprog/${prog}.c
make ${prog} -C tests/accelprog/ W_ACCEL=$W_ACCEL

# FLAG=--debug-flag=VirtualDevice,SimpleCPU
FLAG=--debug-flag=VirtualDevice

echo "========================================================="
echo "cap: $cap; energy: $energy"
build/ARM/gem5.debug $FLAG $script $cap $energy $prog $count $W_ACCEL
# build/ARM/gem5.debug --debug-flag=VirtualDevice --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2
