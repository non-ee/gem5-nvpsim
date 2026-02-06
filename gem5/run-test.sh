rm m5out/batch_res.csv

#echo "========================================================="
#cap=10
#energy=4
#echo "cap: $cap; entergy: $energy"
#build/ARM/gem5.debug configs/example/test_engy_vdev.py $cap $energy
#build/ARM/gem5.debug --debug-flag=VirtualDevice,EnergyMgmt --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2

W_ACCEL=$1

# trace="rf-cart"
trace="solar_10us_30s"
cap=4
profilemult=0.005
prog="image_processing"
# prog="image_processing_16x16"

arg_prog=""
if [ $W_ACCEL -eq 1 ]; then
    arg_prog="sim_w_accel"
    echo "Run with accelerator"
else
    arg_prog="sim_wo_accel"
    echo "Run without accelerator"
fi

script="configs/accel/image_processing/${arg_prog}.py"

make clean -C tests/accelprog/
make ${prog} -C tests/accelprog/ W_ACCEL=$W_ACCEL

# FLAG=--debug-flag=VirtualDevice,SimpleCPU
FLAG=--debug-flag=VirtualDevice,Accelerator
# FLAG=--debug-flag=Accelerator


echo "========================================================="
echo "cap: $cap; energy: $energy"
build/ARM/gem5.debug $FLAG $script $prog $trace $cap $profilemult
# build/ARM/gem5.debug --debug-flag=VirtualDevice --debug-file=virtual_device.o configs/example/sim_exp_br.py $cap $energy -j2
