from ClockedObject import ClockedObject
from m5.params import *


class DmaCtrl(ClockedObject):
    type = "DmaCtrl"
    cxx_header = "accel/dma_ctrl.hh"

    cpu = Param.BaseCPU(NULL, "The cpu of the system")
    bandwidth = Param.UInt32(64, "Transfer size")
    energy_per_tx = VectorParam.Float([], "Energy consumed per transaction")
