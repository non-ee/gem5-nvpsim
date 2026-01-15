from m5.params import *
from MemObject import MemObject

class AccessMonitor(MemObject):
    type = 'AccessMonitor'
    cxx_header = "monitor/access_monitor.hh"

    slave = SlavePort("CPU-side port")
    master = MasterPort("Memory-side port")
