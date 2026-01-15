#include "monitor/access_monitor.hh"
#include "debug/AccessMonitor.hh"

AccessMonitor::AccessMonitor(const Params *p)
    : MemObject(p),
      slavePort(name() + ".slave", *this),
      masterPort(name() + ".master", *this),
      measuring(false)
{
}

/* ===== Port lookup (OLD gem5 style) ===== */

BaseSlavePort&
AccessMonitor::getSlavePort(const std::string &if_name, PortID)
{
    if (if_name == "slave")
        return slavePort;
    return MemObject::getSlavePort(if_name);
}

BaseMasterPort&
AccessMonitor::getMasterPort(const std::string &if_name, PortID)
{
    if (if_name == "master")
        return masterPort;
    return MemObject::getMasterPort(if_name);
}

/* ===== Observation ===== */

void
AccessMonitor::observe(PacketPtr pkt)
{
    DPRINTF(AccessMonitor,
        "Observed access: addr=0x%lx size=%u cmd=%s\n",
        pkt->getAddr(),
        pkt->getSize(),
        pkt->cmdString());
}

/* ===== SlavePort ===== */

AccessMonitor::MonitorSlavePort::MonitorSlavePort(
    const std::string &name, AccessMonitor &o)
    : SlavePort(name, &o), owner(o)
{
}

AddrRangeList
AccessMonitor::MonitorSlavePort::getAddrRanges() const
{
    return AddrRangeList(); // empty → transparent
}

Tick
AccessMonitor::MonitorSlavePort::recvAtomic(PacketPtr pkt)
{
    owner.observe(pkt);
    return owner.masterPort.sendAtomic(pkt);
}

void
AccessMonitor::MonitorSlavePort::recvFunctional(PacketPtr pkt)
{
    owner.observe(pkt);
    owner.masterPort.sendFunctional(pkt);
}

bool
AccessMonitor::MonitorSlavePort::recvTimingReq(PacketPtr pkt)
{
    owner.observe(pkt);
    return owner.masterPort.sendTimingReq(pkt);
}

void
AccessMonitor::MonitorSlavePort::recvRespRetry()
{
    owner.masterPort.sendRetryResp();
}

/* ===== MasterPort ===== */

AccessMonitor::MonitorMasterPort::MonitorMasterPort(
    const std::string &name, AccessMonitor &o)
    : MasterPort(name, &o), owner(o)
{
}

bool
AccessMonitor::MonitorMasterPort::recvTimingResp(PacketPtr pkt)
{
    return owner.slavePort.sendTimingResp(pkt);
}

void
AccessMonitor::MonitorMasterPort::recvReqRetry()
{
    owner.slavePort.sendRetryReq();
}

AccessMonitor *
AccessMonitorParams::create()
{
    return new AccessMonitor(this);
}
