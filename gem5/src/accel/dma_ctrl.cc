#include "accel/dma_ctrl.hh"
#include "accel/mem_if.hh"
#include "dma_ctrl.hh"
#include "mem/se_translating_port_proxy.hh"
#include "cpu/thread_context.hh"
#include "engy/state_machine.hh"
#include "debug/EnergyMgmt.hh"
#include "debug/DmaCtrl.hh"
#include <cstdint>


DmaCtrl::TickEvent::TickEvent(DmaCtrl *c)
    : Event(Accelerator_Tick_Pri), ctrl(c) {}

void DmaCtrl::TickEvent::process() {
    assert(ctrl);
    ctrl->tick();
}

const char *DmaCtrl::TickEvent::description() const {
    return "DMA Tick Event";
}

void DmaCtrl::tick() {
    Tick latency = clockPeriod();

    double EngyConsume = 0;
    EngyConsume = energy_per_tx[dmaTask.op] * ticksToCycles(latency);

    char devname[100] = "DmaCtrl";
    EnergyObject::consumeEnergy(devname, EngyConsume);
    DPRINTF(EnergyMgmt, "DmaCtrl consumed %f energy\n", EngyConsume);
    schedule(tickEvent, curTick() + latency);
}


DmaCtrl::DmaCtrl(const DmaCtrlParams *p)
    : ClockedObject(p),
      tickEvent(this),
      cpu(p->cpu),
      portProxy(nullptr),
      mem(nullptr),
      bandwidth(p->bandwidth),
      dmaTask(),
      backupTask(),
      dmaEvent(this, false, Event::Accelerator_DMA_Pri)
{
    inTask = false;

    energy_per_tx[0] = p->energy_per_tx[0];
    energy_per_tx[1] = p->energy_per_tx[1];
    energy_per_tx[2] = p->energy_per_tx[2];

    portProxy = new SETranslatingPortProxy(
        cpu->getDataPort(),
        cpu->getContext(0)->getProcessPtr(),
        SETranslatingPortProxy::AllocType::Never
    );
    mem = new AccelMemInterface(portProxy);
}

DmaCtrl::~DmaCtrl() {
    if (portProxy)
        delete portProxy;
    if (mem)
        delete mem;
    if (tickEvent.scheduled())
        deschedule(tickEvent);
}

void DmaCtrl::init() {
    DPRINTF(DmaCtrl, "Initializing DmaCtrl\n");

    if (portProxy) {
        DPRINTF(DmaCtrl, "Port proxy created successfully\n");
    } else {
        panic("DmaCtrl::init(): failed to create port proxy\n");
    }

    if (mem) {
        DPRINTF(DmaCtrl, "Memory interface created successfully\n");
    } else {
        panic("DmaCtrl::init(): failed to create memory interface\n");
    }

    if (!tickEvent.scheduled())
        schedule(tickEvent, clockEdge(Cycles(0)));
}

void DmaCtrl::startRead(Addr addr, uint8_t* buf, size_t size, std::function<void()> cb)
{
    DPRINTF(DmaCtrl, "Starting read from address %lx\n", addr);
    inTask = true;
    dmaTask = DmaTask(addr, buf, size, cb, READ);
    schedule(dmaEvent, clockEdge(Cycles(1)));
}

void DmaCtrl::startWrite(Addr addr, uint8_t* buf, size_t size, std::function<void()> cb)
{
    DPRINTF(DmaCtrl, "Starting read from address %lx\n", addr);
    inTask = true;
    dmaTask = DmaTask(addr, buf, size, cb, WRITE);
    schedule(dmaEvent, clockEdge(Cycles(1)));
}

void DmaCtrl::doDma() {
    auto &t = dmaTask;

    if (t.sizeLeft == 0) {
        t.op = OFF;
        inTask = false;
        if (t.cb)
            t.cb();
        return;
    }

    size_t chunk = std::min(bandwidth, t.sizeLeft);

    if (t.op == READ)
        mem->read(t.addr, t.buf, chunk);
    else if (t.op == WRITE)
        mem->write(t.addr, t.buf, chunk);
    else
        panic("Invalid state");

    t.addr += chunk;
    t.buf += chunk;
    t.sizeLeft -= chunk;

    DPRINTF(DmaCtrl, "Access to %lu bytes from address %lx\n", chunk, t.addr);

    schedule(dmaEvent, clockEdge(Cycles(1)));
}

int DmaCtrl::handleMsg(const EnergyMsg& msg) {
    if (!inTask)
        return 1;

    if (msg.type == SimpleEnergySM::MsgType::POWER_OFF) {
        if (!active()) return 1;

        DPRINTF(DmaCtrl, "Powering off ...\n");
        backupDma();
    }
    else if (msg.type == SimpleEnergySM::MsgType::POWER_ON) {
        if (active()) return 1;

        DPRINTF(DmaCtrl, "Powering on ...\n");
        restoreDma();
    }
    else {
        DPRINTF(EnergyMgmt, "Unrecognized MsgType!\n");
        return 0;
    }

    return 1;
}

void DmaCtrl::backupDma() {
    if (dmaEvent.scheduled())
        deschedule(dmaEvent);

    backupTask = dmaTask;
    dmaTask.op = OFF;
}

void DmaCtrl::restoreDma() {
    dmaTask = backupTask;

    if (!dmaEvent.scheduled())
        schedule(dmaEvent, clockEdge(Cycles(1)));
}

DmaCtrl* DmaCtrlParams::create()
{
    return new DmaCtrl(this);
}
