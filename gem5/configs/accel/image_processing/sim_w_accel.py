from pickletools import float8
import m5
from m5.objects import *
import sys
import os

if os.path.exists("m5out/devicedata"):
	os.remove("m5out/devicedata")

if os.path.exists("m5out/power_failure"):
	os.remove("m5out/power_failure")

if os.path.exists("m5out/energy_consumed.txt"):
	os.remove("m5out/energy_consumed.txt")

if os.path.exists("m5out/ticks_output.txt"):
	os.remove("m5out/ticks_output.txt")

system = System()
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1MHz'				# clock period: 1us
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'atomic'
system.mem_ranges = [
    AddrRange('512MB')
]

###################################
#####	Energy Management Profiles #####
###################################
cap = float(sys.argv[3])
profilemult = float(sys.argv[4])
print "cap: %f; energy: %f.\n" %(cap, profilemult)
# cap = cap * 0.2

# Power Supply (file path and sample period)
trace = sys.argv[2]
energy_path = 'profile/%s.txt' % trace
system.energy_mgmt = EnergyMgmt(path_energy_profile = energy_path, energy_time_unit = '10us')
# Energy Management Strategy: State Machine
system.energy_mgmt.state_machine = SimpleEnergySM()
# Threshold Design for the state machine
system.energy_mgmt.state_machine.thres_1_to_off = 0.5 * cap * 1000 * 1.1 * 1.1
system.energy_mgmt.state_machine.thres_off_to_1 = 0.5 * cap * 1000 * 4.5 *4.5
# Energy Storage and leakage design of the capacitor
system.energy_mgmt.capacity = cap;				# uF
system.energy_mgmt.system_leakage = 0.2;			# leakage
system.energy_mgmt.energy_profile_mult = profilemult; 	# adjust the energy

print "---- Full cap: %f." %(0.5 * cap * 1000 * 5 * 5)
print "---- thres_1_to_off: %f." %(system.energy_mgmt.state_machine.thres_1_to_off)
print "---- thres_off_to_1: %f." %(system.energy_mgmt.state_machine.thres_off_to_1)
print "---- deltaE = %f.\n" %(system.energy_mgmt.state_machine.thres_off_to_1 - system.energy_mgmt.state_machine.thres_1_to_off)

###################################
##########	CPU 	###############
###################################

# CPU: basic params
system.cpu = AtomicSimpleCPU(
			power_cpu = [0, 0.3, 1.3], 	# nJ/cycle
			cycle_backup = 5, 		# nJ/cycle
			cycle_restore = 3 		# nJ/cycle
		)
# CPU: slave port
system.cpu.s_energy_port = system.energy_mgmt.m_energy_port
# CPU: memory
system.membus = SystemXBar()
system.cpu.icache_port = system.membus.slave
system.cpu.dcache_port = system.membus.slave
system.cpu.createInterruptController()

system.mem_ctrl = DDR3_1600_x64()
system.mem_ctrl.range = system.mem_ranges[0]
system.mem_ctrl.port = system.membus.master
system.system_port = system.membus.slave

###################################
##########  Virtual Device  ##########
###################################
#vdev
system.has_vdev = 1
system.vdev_ranges = [
    AddrRange('512MB', '512MB'),
    AddrRange('513MB', '513MB')
]
system.vaddr_vdev_ranges = [
    AddrRange('1000MB', '1000MB'),
    AddrRange('1001MB', '1001MB')
]

# Virtual device 1
system.vdev0 = VirtualDevice(id=0)
system.vdev0.cpu = system.cpu
system.vdev0.range = system.vdev_ranges[0]
system.vdev0.energy_consumed_per_cycle_vdev = [Float(0), Float(0), Float(3.35e-3), Float(3.35e-3)]
system.vdev0.delay_set = '100us'
system.vdev0.delay_self = '12us'
system.vdev0.delay_cpu_interrupt = '25us'
system.vdev0.delay_recover = '100us'
system.vdev0.is_interruptable = 0
system.vdev0.port = system.membus.master
system.vdev0.s_energy_port = system.energy_mgmt.m_energy_port
system.vdev0.need_log = 1

## Virtual Device 2: Transmitter
system.vdev1 = VirtualDevice(id=1)
system.vdev1.cpu = system.cpu
system.vdev1.range = system.vdev_ranges[1]
system.vdev1.energy_consumed_per_cycle_vdev = [Float(0), Float(0), Float(9.0), Float(9.0)]
system.vdev1.delay_set = '100us'
system.vdev1.delay_self = '8us'
system.vdev1.delay_cpu_interrupt = '25us'
system.vdev1.delay_recover = '100us'
system.vdev1.is_interruptable = 0
system.vdev1.port = system.membus.master
system.vdev1.s_energy_port = system.energy_mgmt.m_energy_port
system.vdev1.need_log = 1

###########  DMA Controller  ############
system.dma_ctrl = DmaCtrl()
system.dma_ctrl.cpu = system.cpu
system.dma_ctrl.s_energy_port = system.energy_mgmt.m_energy_port
system.dma_ctrl.latency_access_per_byte = "0.56us"
system.dma_ctrl.energy_access_per_byte = Float(0.4)

# Energy for [OFF, READ, WRITE]

###########  Accelerator  ############
system.accel = Accelerator()
system.accel.cpu = system.cpu
system.accel.dma_ctrl = system.dma_ctrl
system.accel.s_energy_port = system.energy_mgmt.m_energy_port
system.accel.ctrl_port = system.membus.master

system.accel_range = AddrRange(0x50000000, size='2MB')
system.accel.control_range = system.accel_range

## Eyeriss config, high performance accel
system.accel.compute_unit = ImageProcessingUnit(latency="0.03us")
system.accel.energy_per_cycle = [Float(0.0), Float(2), Float(0.7), Float(20)]
system.accel.delay_init = '81.76us'
system.accel.delay_cpu_interrupt = '25us'
system.accel.is_interruptable = 0

## MOUSE
# system.accel.compute_unit = ImageProcessingUnit(latency="50us")
# system.accel.energy_per_cycle = [Float(0.0), Float(0.0), Float(0.7), Float(5e-3)]
# system.accel.delay_init = '4.48ns'  # Eyeriss
# system.accel.delay_cpu_interrupt = '25us'
# system.accel.is_interruptable = 1

## Sonic
# system.accel.compute_unit = ImageProcessingUnit(latency="0.35ms")
# system.accel.energy_per_cycle = [Float(0.0), Float(7.43), Float(1.2), Float(74.3)]
# system.accel.delay_init = '50us'
# system.accel.delay_cpu_interrupt = '25us'
# system.accel.is_interruptable = 0


###################################
###########  Benchmark  ############
###################################
process = LiveProcess()
# Benchmark path
prog = sys.argv[1]
process.cmd = ['tests/accelprog/%s' % prog]
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system = False, system = system)
m5.instantiate()

print "Beginning simulation!"
exit_event = m5.simulate(int(9999900000))
print 'Exiting @ tick %i because %s' % (m5.curTick(), exit_event.getCause())

###################################
###########  Output File  ############
###################################

# The following codes are used to batch.

if os.path.exists("m5out/power_failure"):
	fi = open("m5out/power_failure","r")
	line = fi.readline()
	power_failure = int(line)
	fi.close()
else:
	power_failure = 0

fo = open("m5out/batch_res.csv","a")
fo.write("%f,%f,%i,%i,%s\n" % (cap, profilemult, power_failure, m5.curTick(), exit_event.getCause()))
fo.close()

print "%f,%f,%i,%i" % (cap, profilemult, power_failure, m5.curTick())

fo = open("m5out/ticks_output.txt","a")
fo.write("Total simulation ticks: %i\n" % m5.curTick())
fo.close()

#fi = open("m5out/devicedata","r")
#line = fi.readline()
#vdev_access = int(line)
#print "vdev3 access: %i" % vdev_access
#fi.close()
