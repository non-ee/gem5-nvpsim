rm m5out/batch_res.csv

#echo "========================================================="
#cap=10
#energy=4
#echo "cap: $cap; entergy: $energy"
#build/ARM/gem5.debug configs/example/test_engy_vdev.py $cap $energy
#build/ARM/gem5.debug --debug-flag=VirtualDevice,EnergyMgmt --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2

#exit

cap=10
energy=8

echo "========================================================="
echo "cap: $cap; energy: $energy"
build/ARM/gem5.debug configs/example/sim_exp_br.py $cap $energy
#build/ARM/gem5.debug --debug-flag=VirtualDevice --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2

