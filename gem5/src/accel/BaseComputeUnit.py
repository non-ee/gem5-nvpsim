from m5.params import *
from m5.proxy import *
from m5.SimObject import SimObject


class BaseComputeUnit(SimObject):
    type = "BaseComputeUnit"
    cxx_header = "accel/compute_unit.hh"
