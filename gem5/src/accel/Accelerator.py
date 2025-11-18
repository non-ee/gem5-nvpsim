from m5.params import *
from MemObject import MemObject
from AtomicSimpleCPU import AtomicSimpleCPU


class Accelerator(MemObject):
    type = "Accelerator"
    cxx_header = "accel/accel.hh"

    ctrlPort = SlavePort("Slave port of accelerator")
    memPort = MasterPort("Master port of accelerator")

    cpu = Param.BaseCPU(NULL, "The cpu of the system")
    controlRange = Param.AddrRange("1MB", "The control range of the accelerator")
    count = Param.Int(0, "The count of the element data")

    delay_init = Param.Clock("2ms", "The tick delay for initialization")
    delay_compute = Param.Clock("10ms", "The tick delay for computation")
    delay_cpu_interrupt = Param.Clock("1ms", "The tick delay for cpu interrupt")
    energy_compute_per_tick = Param.Float(5, "The power consumption of the accelerator")
    energy_idle_per_tick = Param.Float(
        0.5, "The power consumption of the accelerator when idle"
    )
