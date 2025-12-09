from ClockedObject import ClockedObject
from m5.params import *


class DmaCtrl(ClockedObject):
    type = "DmaCtrl"
    cxx_header = "accel/dma_ctrl.hh"

    cpu = Param.BaseCPU(NULL, "The cpu of the system")
    bandwidth = Param.UInt32(64, "Transfer size")
    energy_per_tx = Param.Float(0.0, "Energy consumed per transaction")
    debug_io = Param.Bool(False, "Enable debug output")
