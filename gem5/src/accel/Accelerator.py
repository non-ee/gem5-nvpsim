from AtomicSimpleCPU import AtomicSimpleCPU
from m5.params import *
from MemObject import MemObject


class Accelerator(MemObject):
    type = "Accelerator"
    cxx_header = "accel/accel.hh"

    ctrl_port = SlavePort("Slave port of accelerator")
    compute_unit = Param.BaseComputeUnit(NULL, "The compute unit of the accelerator")

    cpu = Param.BaseCPU(NULL, "The cpu of the system")
    dma_ctrl = Param.DmaCtrl(NULL, "The dma controller of the accelerator")
    control_range = Param.AddrRange("1MB", "The control range of the accelerator")
    count = Param.Int(0, "The count of the element data")

    delay_init = Param.Clock("2ms", "The tick delay for initialization")
    delay_compute = Param.Clock("10ms", "The tick delay for computation")
    delay_cpu_interrupt = Param.Clock("1ms", "The tick delay for cpu interrupt")
    energy_per_cycle = VectorParam.Float([], "The power consumption of the accelerator")
