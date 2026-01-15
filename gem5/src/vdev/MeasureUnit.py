from m5.params import *
from m5.SimObject import SimObject
from VirtualDevice import VirtualDevice

class MeasureUnit(VirtualDevice):
    type = 'MeasureUnit'
    cxx_class = 'MeasureUnit'
    cxx_header = "vdev/measure_unit.hh"
