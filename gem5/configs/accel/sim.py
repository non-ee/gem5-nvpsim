import m5
from m5.objects import *

import os
if os.path.exists("m5out/devicedata"):
	os.remove("m5out/devicedata")

if os.path.exists("m5out/power_failure"):
	os.remove("m5out/power_failure")


import sys
cap = float(sys.argv[1])
profilemult = float(sys.argv[2])
print "cap: %f; energy: %f.\n" %(cap, profilemult)
cap = cap * 0.2
profilemult = profilemult * 0.005
#cap = 10 		# uF
#profilemult = 10	# times

system = System()
system.clk_domain = SrcClockDomain()
system.clk_domain.clock = '1MHz'				# clock period: 1us
system.clk_domain.voltage_domain = VoltageDomain()
system.mem_mode = 'atomic'
system.mem_ranges = [AddrRange('512MB')]


###################################
#####	Energy Management Profiles #####
###################################

# Power Supply (file path and sample period)
system.energy_mgmt = EnergyMgmt(path_energy_profile = 'profile/solar_new_60000.txt', energy_time_unit = '10us')
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

# Virtual device #0
system.vdev0 = VirtualDevice()
system.vdev0.id = 0;
system.vdev0.cpu = system.cpu
# Access address range for the device
system.vdev0.range = system.vdev_ranges[0]
# The energy consumption of each cycle at power-off, idle and active mode.
system.vdev0.energy_consumed_per_cycle_vdev = [Float(0), Float(0.06), Float(0.6), Float(1.35)]
# Delay of an active task
system.vdev0.delay_self = '1ms'
# Delay of the task returning interrupt
system.vdev0.delay_cpu_interrupt = '20us'
# Initialization delay
system.vdev0.delay_set = '2200us'
# Recovering delay :: ToRemove
system.vdev0.delay_recover = '920us'
# The device is volatile (is_interruptable = 0)
system.vdev0.is_interruptable = 0
# Function and energy interface to connect to the system bus
system.vdev0.port = system.membus.master
system.vdev0.s_energy_port = system.energy_mgmt.m_energy_port
# Generate log file of this device
system.vdev0.need_log = 1


###################################
###########  Accelerator  ############
###################################

system.accel = Accelerator()
system.accel.cpu = system.cpu
system.accel.s_energy_port = system.energy_mgmt.m_energy_port
system.accel.ctrlPort = system.membus.master
system.accel.memPort = system.membus.slave

system.accel_range = AddrRange('514MB', '516MB')
system.accel_vaddr = Addr('0x40000000')
system.accel.controlRange = system.accel_range

system.accel.count = 100
system.accel.delay_init = '100us'
system.accel.delay_compute = '4ms'
system.accel.delay_cpu_interrupt = '100us'
system.accel.energy_idle_per_tick = Float(0.4)
system.accel.energy_compute_per_tick = Float(5.0)

###################################
###########  Benchmark  ############
###################################

process = LiveProcess()
# Benchmark path
# process.cmd = ['tests/test-progs/brgMonitor/main_trans_cluster2']
process.cmd = ['tests/accelprog/accel_test']
system.cpu.workload = process
system.cpu.createThreads()

root = Root(full_system = False, system = system)
m5.instantiate()

print "Beginning simulation!"
exit_event = m5.simulate(int(599900000))
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

#fi = open("m5out/devicedata","r")
#line = fi.readline()
#vdev_access = int(line)
#print "vdev3 access: %i" % vdev_access
#fi.close()
