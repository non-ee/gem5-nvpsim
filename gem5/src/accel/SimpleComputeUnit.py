from BaseComputeUnit import BaseComputeUnit
from m5.params import *


class SimpleComputeUnit(BaseComputeUnit):
    type = "SimpleComputeUnit"
    cxx_header = "accel/compute_unit.hh"
    latency = Param.Clock("10ms", "Latency of the compute unit")
