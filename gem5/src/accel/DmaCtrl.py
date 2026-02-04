from ClockedObject import ClockedObject
from m5.params import *


class DmaCtrl(ClockedObject):
    type = "DmaCtrl"
    cxx_header = "accel/dma_ctrl.hh"

    cpu = Param.BaseCPU(NULL, "The cpu of the system")
    latency_access_per_byte = Param.Clock("10ms", "Latency per memory access")
    energy_access_per_byte = Param.Float(
        Float(50.0), "Energy consumed per memory access"
    )
